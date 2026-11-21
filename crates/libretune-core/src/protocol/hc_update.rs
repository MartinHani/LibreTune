//! HaniCore firmware update over the K-line, with the tune kept.
//!
//! A port of the HaniCore repository's bench-proven `tools/flash/hc_update.py`
//! and the field migration of `tools/calibration/me711_migrate_calibration.py`:
//! same protocol, same timing, same files in the output folder. The ECU's side
//! is `hcdiag711.c` / `hcupd711.h`.
//!
//! 1. identity: a wrong board is refused, so is an ECU with no updater;
//! 2. the tune is SAVED to `<out>/page_before.bin` before anything moves;
//! 3. STAGE_BEGIN, the app in 64-byte WRITEs, SEAL (CRC-32), COMMIT. Every '!'
//!    command answers OK whatever it did, so the verdict is read from the
//!    updater status channel, and a chunk whose reply was lost is never
//!    programmed twice: the ECU's received-count says where to resume;
//! 4. the resident loader copies STAGING over the APP; the ECU must come back
//!    as the image's build;
//! 5. same layout: the tune is still there, read back and compared. Layout
//!    changed: the saved page is migrated by field name, written, read back,
//!    validated by the firmware, Burned.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::io;
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::{Duration, Instant};

use regex::bytes::Regex as BytesRegex;
use regex::Regex;

use super::kline::{envelope, kwp_entry_once, KWire};
use super::serial::{configure_port, open_port};

const APP_BASE: usize = 0x030000; // SA6: the app's vector table, header and code
const HDR_OFF: usize = 0x030200; // the 32-byte header the loader reads
const HDR_MAGIC0: u16 = 0x4348; // 'HC'
const HDR_MAGIC1: u16 = 0x5041; // 'AP'
const HDR_SEAL: u16 = 0x5AA5;
const STAGE_CHUNK: u32 = 64; // bounded by hcts.c RXMAX = 72

const UPD_OCH: u16 = 478;
const UPD_OCH_LEN: usize = 18;
const UPD_MARKER: u16 = 0x5550; // 'UP' at OCH 490: this image has the updater

const ST_IDLE: u16 = 0;
const ST_ERASING: u16 = 1;
const ST_READY: u16 = 2;
const ST_WRITING: u16 = 3;
const ST_SEALED: u16 = 4;
const ST_FAILED: u16 = 6;
const ST_NAME: [&str; 7] = [
    "IDLE", "ERASING", "READY", "WRITING", "SEALED", "COMMIT", "FAILED",
];
const ERR_NAME: [&str; 11] = [
    "NONE",
    "RAM",
    "VALIDATE (engine not stopped)",
    "STAGE",
    "ERASE",
    "PROGRAM",
    "VERIFY",
    "RETRIES",
    "LAYOUT",
    "FULL",
    "DEADLINE",
];
const CHECK_EVERY: u32 = 32; // chunks between received-count checks (2 KiB)
const QUEUE_HIGH: u16 = 2048; // wait while the ECU holds more unprogrammed bytes
const LINK_RATE: u32 = 124800; // falls back to 57600
const BULK_RATE: u32 = 250000;
const GAP: Duration = Duration::from_millis(1); // quiet time after each ECU reply
const PORT_TIMEOUT: Duration = Duration::from_millis(5);

const fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn st_name(s: u16) -> &'static str {
    ST_NAME.get(s as usize).copied().unwrap_or("?")
}
fn err_name(e: u16) -> String {
    ERR_NAME
        .get(e as usize)
        .map(|s| s.to_string())
        .unwrap_or_else(|| e.to_string())
}

/// A lost frame can be retried; anything else ends the update.
#[derive(Debug)]
enum E {
    Link(String),
    Fatal(String),
}
impl From<E> for String {
    fn from(e: E) -> String {
        match e {
            E::Link(s) | E::Fatal(s) => s,
        }
    }
}
fn fatal<T>(s: impl Into<String>) -> Result<T, E> {
    Err(E::Fatal(s.into()))
}
fn io_fatal(what: &str, e: io::Error) -> E {
    E::Fatal(format!("{what}: {e}"))
}

// ---------------------------------------------------------------- the image

/// The app region of a 1 MiB HaniCore image, checked against its own header.
#[derive(Debug, Clone)]
pub struct AppImage {
    pub board: u16,
    pub length: usize,
    pub crc: u32,
    pub build_id: String,
    pub body: Vec<u8>,
    pub signature: String,
    pub sig_board: String,
    pub layout: String,
    pub version: String,
    pub board_text: String,
}

fn latin1(b: &[u8]) -> String {
    b.iter().map(|&c| c as char).collect()
}
fn u16le(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// The board name in the version text ("... / Bosch ME7.1.1 4F build X"). The
/// signature's board token is shared by boards with one page layout (the
/// Porsche and the 4F), so this is what tells them apart.
fn board_text(version: &str) -> String {
    Regex::new(r" / (.*?) build [0-9A-F]{8}")
        .unwrap()
        .captures(version)
        .map(|c| c[1].to_string())
        .unwrap_or_default()
}

/// Validate an image before a byte reaches the ECU.
pub fn parse_app(image: &[u8]) -> Result<AppImage, String> {
    if image.len() != 0x100000 {
        return Err(format!(
            "image is {} bytes, not a 1 MiB ECU image",
            image.len()
        ));
    }
    let h = HDR_OFF;
    if u16le(image, h) != HDR_MAGIC0 || u16le(image, h + 2) != HDR_MAGIC1 {
        return Err(
            "no 'HCAP' app header at 0x030200 - not an image with the K-line updater".into(),
        );
    }
    if u16le(image, h + 30) != HDR_SEAL {
        return Err("app header is not sealed (0x5AA5 missing)".into());
    }
    let board = u16le(image, h + 4);
    if !(1..=5).contains(&board) {
        return Err(format!("app header board code {board} is not known"));
    }
    let length = u32le(image, h + 10) as usize;
    let hdr_crc = u32le(image, h + 14);
    let build = u32le(image, h + 18);
    if !(HDR_OFF - APP_BASE + 32..=0x40000).contains(&length) {
        return Err(format!(
            "app length 0x{length:X} is not within (header, 4 sectors]"
        ));
    }
    let body = image[APP_BASE..APP_BASE + length].to_vec();
    // CRC-32 over the app region EXCLUDING its own 32-byte header
    let mut hasher = crc32fast::Hasher::new();
    hasher.update(&image[APP_BASE..HDR_OFF]);
    hasher.update(&image[HDR_OFF + 32..APP_BASE + length]);
    let crc = hasher.finalize();
    if crc != hdr_crc {
        return Err(format!(
            "app CRC-32 0x{crc:08X} != header 0x{hdr_crc:08X} - the image is inconsistent with its own header"
        ));
    }
    let sig_re = BytesRegex::new(r"(?-u)(?:rusEFI )?HaniCore (\w+) ([0-9A-F]{8})").unwrap();
    let ver_re = BytesRegex::new(r"(?s-u)HaniCore V[^\x00]*? build ([0-9A-F]{8})").unwrap();
    let (sig, ver) = match (sig_re.captures(&body), ver_re.captures(&body)) {
        (Some(s), Some(v)) => (s, v),
        _ => return Err("no signature / version text in the app".into()),
    };
    let build_id = format!("{build:08X}");
    if ver[1] != *build_id.as_bytes() {
        return Err(format!(
            "version text build {} != header build id {build_id}",
            latin1(&ver[1])
        ));
    }
    let version = latin1(&ver[0]);
    Ok(AppImage {
        board,
        length,
        crc,
        build_id,
        signature: latin1(&sig[0]),
        sig_board: latin1(&sig[1]),
        layout: latin1(&sig[2]),
        board_text: board_text(&version),
        version,
        body: body.clone(),
    })
}

/// WRITE offsets/sizes covering the whole app, word-aligned, never straddling
/// a 64 KiB segment. The seal word is programmed by SEAL, not here.
fn plan_chunks(length: u32) -> Vec<(u32, u32)> {
    let (mut off, mut out) = (0, Vec::new());
    while off < length {
        let mut n = STAGE_CHUNK.min(length - off).min(0x10000 - (off & 0xFFFF));
        if n & 1 == 1 {
            n += 1; // round up to a word; the tail is 0xFF
        }
        out.push((off, n));
        off += n;
    }
    out
}

// -------------------------------------------------------------- the ECU link

/// The TunerStudio CRC envelope over a K-line adapter. The adapter echoes what
/// is sent (half duplex); the echo is stripped when it is there.
struct Link<W: KWire> {
    wire: Option<W>, // taken for the length of a KWP entry
    last_rx: Instant,
    base_baud: u32,
}

impl<W: KWire> Link<W> {
    fn new(wire: W, baud: u32) -> Result<Self, E> {
        let mut l = Link {
            wire: Some(wire),
            last_rx: Instant::now(),
            base_baud: baud,
        };
        // set once: on Windows every assignment reconfigures the port (~8 ms)
        l.w()
            .set_timeout(PORT_TIMEOUT)
            .map_err(|e| io_fatal("serial port", e))?;
        l.w()
            .set_baud(baud)
            .map_err(|e| io_fatal("serial port", e))?;
        Ok(l)
    }

    fn w(&mut self) -> &mut W {
        self.wire
            .as_mut()
            .expect("wire is only away during a KWP entry")
    }

    fn set_baud(&mut self, baud: u32) -> Result<(), E> {
        self.w()
            .set_baud(baud)
            .map_err(|e| io_fatal("serial port", e))?;
        let _ = self.w().clear_input();
        Ok(())
    }

    /// 5-baud 0x01, the ISO 14230 handshake, HaniCore IDENTIFY and GOFAST; the
    /// ECU then runs a private session at the returned rate.
    fn kwp_enter(&mut self, rate: u32, say: &mut dyn FnMut(&str), tries: u32) -> Result<u32, E> {
        for n in 0..tries {
            let wire = self.wire.take().expect("wire present");
            let (mut wire, r) = kwp_entry_once(wire, rate);
            let _ = wire.set_timeout(PORT_TIMEOUT);
            self.wire = Some(wire);
            match r {
                Ok(got) => {
                    self.base_baud = got;
                    self.last_rx = Instant::now();
                    let _ = self.w().clear_input();
                    return Ok(got);
                }
                Err(e) => say(&format!("  kwp: {e}")),
            }
            if n + 1 < tries {
                sleep(ms(2500)); // the ECU drops back to KWP idle on silence
            }
        }
        Err(E::Link(format!("no KWP entry after {tries} tries")))
    }

    fn write_all(&mut self, mut data: &[u8]) -> Result<(), E> {
        // the port's short timeout also bounds a write: it may take several calls
        let end = Instant::now() + ms(2000);
        while !data.is_empty() {
            match self.w().write(data) {
                Ok(n) => data = &data[n..],
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => return Err(E::Link(format!("serial write: {e}"))),
            }
            if !data.is_empty() && Instant::now() > end {
                return Err(E::Link("serial write timed out".into()));
            }
        }
        self.w()
            .flush()
            .map_err(|e| E::Link(format!("serial flush: {e}")))
    }

    /// Send one frame; the response data (code stripped), or a link error.
    /// `data_len` None = do not wait for a reply (COMMIT resets).
    fn request(
        &mut self,
        payload: &[u8],
        data_len: Option<usize>,
        timeout: Duration,
        idle: Duration,
    ) -> Result<Vec<u8>, E> {
        // The ECU drops one received byte per byte it transmitted as its own
        // echo; its 1 ms tick must have eaten the last before our first lands.
        let since = self.last_rx.elapsed();
        if since < GAP {
            sleep(GAP - since);
        }
        let req = envelope(payload);
        let _ = self.w().clear_input();
        self.write_all(&req)?;
        let Some(data_len) = data_len else {
            return Ok(Vec::new());
        };
        let want = req.len() + data_len + 7;
        let mut got = vec![0u8; want];
        let mut have = 0;
        let mut deadline = Instant::now() + timeout;
        while Instant::now() < deadline && have < want {
            // ask for exactly what is still owed; a read returns what has
            // arrived, so a short reply costs `idle`, not the whole timeout
            match self.w().read(&mut got[have..]) {
                Ok(0) => {}
                Ok(n) => {
                    have += n;
                    deadline = deadline.min(Instant::now() + idle);
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => return Err(E::Link(format!("serial read: {e}"))),
            }
        }
        self.last_rx = Instant::now();
        got.truncate(have);
        let tag = latin1(&payload[..payload.len().min(2)]);
        // the echo is our own frame; at 250 kbaud the adapter sometimes garbles
        // it while the reply behind it is intact - the reply's CRC decides
        let raw: &[u8] = if got.starts_with(&req) || got.len() >= req.len() + 7 {
            &got[req.len().min(got.len())..]
        } else {
            &got
        };
        if raw.len() < 7 {
            return Err(E::Link(format!(
                "no reply to {tag:?} ({} bytes)",
                raw.len()
            )));
        }
        let n = u16::from_be_bytes([raw[0], raw[1]]) as usize;
        if n == 0 || raw.len() < 2 + n + 4 {
            return Err(E::Link(format!("bad reply frame to {tag:?}")));
        }
        let blob = &raw[2..2 + n];
        let crc = u32::from_be_bytes([raw[2 + n], raw[3 + n], raw[4 + n], raw[5 + n]]);
        if crc != crc32fast::hash(blob) {
            return Err(E::Link(format!("bad reply frame to {tag:?}")));
        }
        if blob[0] != 0 {
            return Err(E::Link(format!(
                "response code 0x{:02X} to {tag:?}",
                blob[0]
            )));
        }
        Ok(blob[1..].to_vec())
    }

    fn retry(
        &mut self,
        payload: &[u8],
        data_len: usize,
        timeout: Duration,
        idle: Duration,
    ) -> Result<Vec<u8>, E> {
        let mut last = E::Link("no try".into());
        for _ in 0..4 {
            match self.request(payload, Some(data_len), timeout, idle) {
                Ok(v) => return Ok(v),
                Err(e) => last = e,
            }
            sleep(ms(100));
        }
        Err(last)
    }

    fn cmd(&mut self, payload: &[u8]) -> Result<Vec<u8>, E> {
        self.retry(payload, 0, ms(2000), ms(250))
    }

    fn text(&mut self, cmd: &[u8]) -> Result<String, E> {
        let blob = self.retry(cmd, 80, ms(2000), ms(150))?;
        let end = blob.iter().position(|&b| b == 0).unwrap_or(blob.len());
        Ok(latin1(&blob[..end]).trim().to_string())
    }

    fn och(&mut self, off: u16, n: usize) -> Result<Vec<u8>, E> {
        let mut req = vec![b'O'];
        req.extend_from_slice(&off.to_be_bytes());
        req.extend_from_slice(&(n as u16).to_be_bytes());
        // an OCH reply takes a few ms; a lost one should cost little
        let blob = self.retry(&req, n, ms(100), ms(20))?;
        if blob.len() != n {
            return Err(E::Link(format!(
                "OCH @{off} returned {} of {n} bytes",
                blob.len()
            )));
        }
        Ok(blob)
    }

    fn och16(&mut self, off: u16) -> Result<u16, E> {
        let b = self.och(off, 2)?;
        Ok(u16::from_be_bytes([b[0], b[1]]))
    }

    fn raw_status(&mut self) -> Result<Status, E> {
        Ok(Status::decode(&self.och(UPD_OCH, UPD_OCH_LEN)?))
    }

    fn page_read(&mut self, size: usize) -> Result<Vec<u8>, E> {
        let mut out = Vec::with_capacity(size);
        let mut off = 0;
        while off < size {
            let n = 256.min(size - off);
            let mut req = vec![b'R'];
            req.extend_from_slice(&(off as u16).to_be_bytes());
            req.extend_from_slice(&(n as u16).to_be_bytes());
            let blob = self.retry(&req, n, ms(3000), ms(250))?;
            if blob.len() != n {
                return Err(E::Link(format!("page read @{off} short")));
            }
            out.extend_from_slice(&blob);
            off += n;
        }
        Ok(out)
    }

    fn page_write(&mut self, off: usize, data: &[u8]) -> Result<(), E> {
        let mut req = vec![b'W'];
        req.extend_from_slice(&(off as u16).to_be_bytes());
        req.extend_from_slice(data);
        self.cmd(&req).map(|_| ())
    }
}

#[derive(Debug, Clone, Copy)]
struct Status {
    state: u16,
    error: u16,
    received: u32,
    seq: u8,
    rc: u8,
    cmd: u8,
    marker: u16,
    pending: u16,
    erased: u16,
}

impl Status {
    fn decode(b: &[u8]) -> Status {
        let w = |i: usize| u16::from_be_bytes([b[2 * i], b[2 * i + 1]]);
        Status {
            state: w(0),
            error: w(1),
            received: ((w(2) as u32) << 16) | w(3) as u32,
            seq: (w(4) >> 8) as u8,
            rc: (w(4) & 0xFF) as u8,
            cmd: (w(5) & 0xFF) as u8,
            marker: w(6),
            pending: w(7),
            erased: w(8),
        }
    }
}

// ---------------------------------------------------------------- the driver

struct Updater<'a, W: KWire> {
    k: Link<W>,
    say: &'a mut dyn FnMut(&str),
    kwp: bool,
    fast: bool,
    want_fast: bool,
    erase_seq: Option<u8>,
    erase_t0: Instant,
    resends: i64,
}

impl<W: KWire> Updater<'_, W> {
    fn say(&mut self, m: &str) {
        (self.say)(m)
    }

    /// The status, riding through a silent link. A lost status is harmless:
    /// nothing in the ECU moves without the next command. After 0.4 s of
    /// silence at the bulk rate the ECU's link hunter has most likely left it.
    fn status(&mut self) -> Result<Status, E> {
        let t0 = Instant::now();
        let mut pause = ms(20);
        loop {
            match self.k.raw_status() {
                Ok(st) => {
                    if t0.elapsed() > ms(1000) {
                        let m = format!(
                            "link back after {:.1} s of silence",
                            t0.elapsed().as_secs_f32()
                        );
                        self.say(&m);
                    }
                    return Ok(st);
                }
                Err(E::Link(e)) => {
                    if t0.elapsed() > Duration::from_secs(30) {
                        return Err(E::Link(e));
                    }
                    if self.fast && t0.elapsed() > ms(400) {
                        self.recover_link()?;
                        continue;
                    }
                    sleep(pause);
                    pause = (pause * 2).min(ms(500));
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Find the ECU at the rates its hunter cycles through, then go back to
    /// the bulk rate. On the vehicle path there is no hunt: the session is
    /// entered again, the ECU's update state is untouched, and the bulk rate
    /// is not tried again - one failure on a car's wiring is the answer.
    fn recover_link(&mut self) -> Result<(), E> {
        self.fast = false;
        if self.kwp {
            sleep(ms(2500));
            let base = self.k.base_baud;
            let rate = self.k.kwp_enter(base, &mut *self.say, 3)?;
            self.say(&format!("vehicle session re-entered at {rate} baud"));
            self.want_fast = false;
            return Ok(());
        }
        for _ in 0..4 {
            for baud in [self.k.base_baud, 57600] {
                self.k.set_baud(baud)?;
                for _ in 0..3 {
                    if self.k.raw_status().is_ok() {
                        self.say(&format!("link found again at {baud} baud"));
                        if self.want_fast {
                            self.go_fast()?;
                        }
                        return Ok(());
                    }
                }
            }
        }
        Err(E::Link(format!(
            "the ECU does not answer at {} or 57600",
            self.k.base_baud
        )))
    }

    /// Erase STAGING (sliced like a store reclaim). Started early so the tune
    /// is saved while it runs: the erase touches neither the store nor the page.
    fn begin_erase(&mut self, st: Status) -> Result<(), E> {
        if st.marker != UPD_MARKER {
            return fatal(format!(
                "the ECU publishes no updater status (OCH 490 = 0x{:04X}): it needs one bootmode flash of an image with the updater",
                st.marker
            ));
        }
        if st.state == ST_ERASING || st.state == ST_WRITING {
            return fatal(format!(
                "an update is already in progress ({})",
                st_name(st.state)
            ));
        }
        self.k.cmd(b"!u")?;
        self.erase_seq = Some(st.seq);
        self.erase_t0 = Instant::now();
        // read the verdict now: the next command ('!b') overwrites it, and a
        // refusal (engine running) would then report "REFUSED: NONE"
        let end = Instant::now() + ms(500);
        while Instant::now() < end {
            if let Ok(s) = self.k.raw_status() {
                if s.seq != st.seq && s.cmd == b'u' {
                    if s.rc != 0 {
                        return fatal(format!(
                            "ECU REFUSED STAGE_BEGIN: {}",
                            err_name(s.rc as u16)
                        ));
                    }
                    break;
                }
            }
        }
        Ok(())
    }

    /// '!b': the ECU moves to 250 kbaud once the reply has left the wire.
    /// Follow it, prove it with a status read; if the line will not hold the
    /// rate, go back - the ECU's link hunter follows the host back down.
    fn go_fast(&mut self) -> Result<bool, E> {
        self.want_fast = true;
        if matches!(self.k.cmd(b"!b"), Err(E::Link(_))) {
            return Ok(false);
        }
        sleep(ms(30));
        self.k.set_baud(BULK_RATE)?;
        let end = Instant::now() + ms(500);
        while Instant::now() < end {
            match self.k.raw_status() {
                Ok(st) if st.cmd == b'b' && st.rc == 0 => {
                    self.say(&format!("link at {BULK_RATE} baud for the transfer"));
                    self.fast = true;
                    return Ok(true);
                }
                Ok(_) => {}
                Err(_) => sleep(ms(50)),
            }
        }
        let base = self.k.base_baud;
        self.k.set_baud(base)?;
        self.say(&format!(
            "the line did not hold {BULK_RATE} baud - staying at {base}"
        ));
        Ok(false)
    }

    fn stage(&mut self, app: &AppImage, t0: Instant) -> Result<(), E> {
        let erase_seq = self.erase_seq.expect("begin_erase ran");
        // wait for READY - judged by state, not by the last command letter
        // ('!b' may have followed the '!u')
        let end = Instant::now() + Duration::from_secs(120);
        loop {
            let st = self.status()?;
            if st.seq != erase_seq && st.state == ST_READY {
                break;
            }
            if st.state == ST_FAILED {
                return fatal(format!(
                    "ECU FAILED erasing STAGING: {}",
                    err_name(st.error)
                ));
            }
            if st.seq != erase_seq && st.state == ST_IDLE {
                return fatal(format!(
                    "ECU REFUSED STAGE_BEGIN: {}",
                    err_name(st.rc as u16)
                ));
            }
            if Instant::now() > end {
                return fatal(format!("timeout erasing STAGING (last status {st:?})"));
            }
            sleep(ms(20));
        }
        let m = format!(
            "STAGING erased in {:.1} s",
            self.erase_t0.elapsed().as_secs_f32()
        );
        self.say(&m);
        let chunks = plan_chunks(app.length as u32);
        let &(last_off, last_n) = chunks.last().expect("the app is not empty");
        let total = last_off + last_n;
        self.stream(&app.body, &chunks, t0)?;
        let st = self.settle(total)?;
        let m = format!(
            "STAGING written: {total} bytes in {:.1} s ({} resends)",
            t0.elapsed().as_secs_f32(),
            self.resends
        );
        self.say(&m);
        // SEAL: the ECU's running CRC must equal the image's
        let mut req = b"!x".to_vec();
        req.extend_from_slice(&app.crc.to_be_bytes());
        self.k.cmd(&req)?;
        let end = Instant::now() + ms(10000);
        loop {
            if Instant::now() > end {
                return fatal("timeout waiting for SEAL");
            }
            if let Ok(s) = self.k.raw_status() {
                if s.seq != st.seq && s.cmd == b'x' {
                    if s.rc != 0 {
                        return fatal(format!(
                            "ECU REFUSED SEAL: {} (state {})",
                            err_name(s.rc as u16),
                            st_name(s.state)
                        ));
                    }
                    if s.state == ST_FAILED {
                        return fatal(format!("ECU FAILED during SEAL: {}", err_name(s.error)));
                    }
                    if s.state == ST_SEALED {
                        break;
                    }
                }
            }
            sleep(ms(5));
        }
        self.say(&format!(
            "STAGING sealed: CRC-32 0x{:08X} matched on the ECU",
            app.crc
        ));
        Ok(())
    }

    /// Let the ECU's job buffer drain; the chunk index it wants next.
    fn drained_at(
        &mut self,
        chunks: &[(u32, u32)],
        index: &HashMap<u32, usize>,
    ) -> Result<usize, E> {
        let end = Instant::now() + ms(5000);
        let mut st = self.status()?;
        while st.state == ST_WRITING && Instant::now() < end {
            st = self.status()?;
        }
        if st.state == ST_FAILED {
            return fatal(format!("ECU FAILED while writing: {}", err_name(st.error)));
        }
        let (o, n) = chunks[chunks.len() - 1];
        if st.received == o + n {
            return Ok(chunks.len());
        }
        match index.get(&st.received) {
            Some(&i) => Ok(i),
            None => fatal(format!(
                "ECU received {} bytes - not a chunk boundary",
                st.received
            )),
        }
    }

    fn queued_at(
        &mut self,
        chunks: &[(u32, u32)],
        index: &HashMap<u32, usize>,
    ) -> Result<usize, E> {
        let end = Instant::now() + ms(5000);
        let mut st = self.status()?;
        while st.state == ST_WRITING && st.pending > QUEUE_HIGH && Instant::now() < end {
            st = self.status()?;
        }
        if st.state == ST_FAILED {
            return fatal(format!("ECU FAILED while writing: {}", err_name(st.error)));
        }
        match index.get(&st.received) {
            Some(&i) => Ok(i),
            None => self.drained_at(chunks, index),
        }
    }

    /// Send the chunks back to back: the ECU queues a chunk that arrives while
    /// it still programs the last, and accepts strictly in order. Every
    /// CHECK_EVERY chunks (and before each new 64 KiB segment, which it cannot
    /// append across) the host reads the received count and resends from
    /// there. Nothing is ever programmed twice.
    fn stream(&mut self, body: &[u8], chunks: &[(u32, u32)], t0: Instant) -> Result<(), E> {
        let index: HashMap<u32, usize> = chunks.iter().enumerate().map(|(i, c)| (c.0, i)).collect();
        self.resends = 0;
        let (mut i, mut since, mut shown) = (0usize, 0u32, usize::MAX);
        while i < chunks.len() {
            let (off, n) = chunks[i];
            if since != 0 && off != 0 && off & 0xFFFF == 0 {
                since = 0; // a new 64 KiB segment cannot be appended to the old job
                let j = self.drained_at(chunks, &index)?;
                if j != i {
                    self.resends += i as i64 - j as i64;
                    i = j;
                    continue;
                }
            }
            if off & 0xFFFF == 0 {
                // that sector may still be erasing in the background
                let end = Instant::now() + Duration::from_secs(30);
                let mut st = self.status()?;
                while st.erased as u32 <= off >> 16 && Instant::now() < end {
                    if st.state == ST_FAILED {
                        return fatal(format!(
                            "ECU FAILED erasing STAGING: {}",
                            err_name(st.error)
                        ));
                    }
                    sleep(ms(10));
                    st = self.status()?;
                }
            }
            let mut req = b"!w".to_vec();
            req.extend_from_slice(&off.to_be_bytes());
            let from = off as usize;
            let to = (from + n as usize).min(body.len());
            req.extend_from_slice(&body[from..to]);
            req.resize(6 + n as usize, 0xFF);
            // a reply takes ~8 ms at 250 kbaud: waiting longer for one that is
            // not coming costs more than the resend
            let lost = matches!(
                self.k.request(&req, Some(0), ms(20), ms(10)),
                Err(E::Link(_))
            );
            i += 1;
            since += 1;
            if lost || since >= CHECK_EVERY || i == chunks.len() {
                since = 0;
                let j = if i < chunks.len() {
                    self.queued_at(chunks, &index)?
                } else {
                    self.drained_at(chunks, &index)?
                };
                if j != i {
                    self.resends += i as i64 - j as i64;
                    i = j;
                }
            }
            if i != shown && (i % 512 == 0 || i == chunks.len()) {
                shown = i;
                let m = format!(
                    "  {i:4}/{} chunks  {:5.1} s",
                    chunks.len(),
                    t0.elapsed().as_secs_f32()
                );
                self.say(&m);
            }
        }
        Ok(())
    }

    fn settle(&mut self, end_at: u32) -> Result<Status, E> {
        let end = Instant::now() + ms(10000);
        loop {
            let st = self.status()?;
            if st.state == ST_FAILED {
                return fatal(format!("ECU FAILED while writing: {}", err_name(st.error)));
            }
            if st.state == ST_READY {
                if st.received == end_at {
                    return Ok(st);
                }
                return fatal(format!("ECU is at byte {}, expected {end_at}", st.received));
            }
            if Instant::now() > end {
                return fatal(format!(
                    "ECU still {} at byte {}",
                    st_name(st.state),
                    st.received
                ));
            }
            sleep(ms(5));
        }
    }

    fn commit(&mut self) -> Result<(), E> {
        let st = self.status()?;
        if st.state != ST_SEALED {
            return fatal(format!(
                "not SEALED ({}) - refusing to commit",
                st_name(st.state)
            ));
        }
        self.k.request(b"!y", None, ms(0), ms(0)).map(|_| ()) // SRST: no reply
    }

    /// The ECU resets into the loader, which copies STAGING over the APP. If
    /// it is not back within 30 s, a key cycle does the same: the request is
    /// in battery-backed SRAM. Only a lost KL30 drops it - then the old build
    /// runs, the tune is untouched, and the update is simply run again.
    fn wait_reboot(&mut self, build: &str) -> Result<String, E> {
        let base = self.k.base_baud;
        self.k.set_baud(base)?; // the reset boots the ECU at its default rate
        self.fast = false;
        let t0 = Instant::now();
        let (auto, end) = (Duration::from_secs(30), Duration::from_secs(330));
        let (mut asked, mut last) = (false, String::new());
        let want = format!("build {build}");
        sleep(ms(1000));
        while t0.elapsed() < end {
            if self.kwp {
                // the reset ended the vehicle session: enter it again
                let _ = self.k.kwp_enter(base, &mut |_| {}, 1);
            }
            if let Ok(v) = self.k.text(b"V") {
                if v.contains(&want) {
                    return Ok(v);
                }
                last = v;
            }
            if !asked && t0.elapsed() > auto {
                self.say("the ECU is not back yet: switch the IGNITION OFF, wait 10 s, switch it ON (keep the battery connected)");
                asked = true;
            }
            sleep(ms(500));
        }
        fatal(format!(
            "the ECU did not come back as build {build} (last {last:?}). If it reports the OLD build, the tune is untouched - run the update again"
        ))
    }

    /// Write the bytes that differ, read back, require the firmware's own
    /// validation, Burn. `force_burn`: after a layout change the page can come
    /// back intact from battery-backed RAM with no flash record of the new
    /// layout yet - a battery disconnect would then lose the tune.
    fn restore_page(
        &mut self,
        want: &[u8],
        ini_text: &str,
        force_burn: bool,
    ) -> Result<(usize, bool), E> {
        let have = self.k.page_read(want.len())?;
        let (diff, runs) = diff_runs(&have, want, 16);
        if diff == 0 && !force_burn {
            return Ok((0, false));
        }
        if diff != 0 {
            self.say(&format!("writing {diff} bytes in {} runs", runs.len()));
            for (off, n) in &runs {
                self.k.page_write(*off, &want[*off..*off + *n])?;
            }
            sleep(ms(500));
        }
        if self.k.page_read(want.len())? != want {
            return fatal("page read-back differs from the restored page - NOT burning");
        }
        let ch = och_channels(ini_text);
        let need = |name: &str| {
            ch.get(name)
                .copied()
                .ok_or_else(|| E::Fatal(format!("the .ini has no channel {name}")))
        };
        let val = self.k.och16(need("cfgValidation")?)?;
        if val != 0 {
            return fatal(format!(
                "the firmware does not validate the restored page (cfgValidation 0x{val:04X}) - NOT burning"
            ));
        }
        // the reply is deferred until the record is programmed (~3 s)
        self.k.request(
            b"B",
            Some(0),
            Duration::from_secs(20),
            Duration::from_secs(8),
        )?;
        let nvm_off = need("nvmState")?;
        let mut nvm = None;
        for _ in 0..60 {
            let v = self.k.och16(nvm_off)?;
            nvm = Some(v);
            if matches!(v & 0xFF, 0 | 5 | 6) {
                break;
            }
            sleep(ms(500));
        }
        match nvm {
            Some(v) if matches!(v & 0xFF, 0 | 5) => Ok((diff, true)),
            v => fatal(format!(
                "Burn did not complete (nvmState 0x{:04X})",
                v.unwrap_or(0)
            )),
        }
    }
}

fn diff_runs(have: &[u8], want: &[u8], limit: usize) -> (usize, Vec<(usize, usize)>) {
    let diff: Vec<usize> = (0..want.len()).filter(|&i| have[i] != want[i]).collect();
    let (mut runs, mut i) = (Vec::new(), 0);
    while i < diff.len() {
        let mut j = i;
        while j + 1 < diff.len() && diff[j + 1] == diff[j] + 1 && diff[j + 1] - diff[i] < limit {
            j += 1;
        }
        runs.push((diff[i], diff[j] - diff[i] + 1));
        i = j + 1;
    }
    (diff.len(), runs)
}

// ------------------------------------------------------------ the .ini facts

fn ini_page_size(text: &str) -> usize {
    Regex::new(r"(?m)^\s*pageSize\s*=\s*(\d+)")
        .unwrap()
        .captures(text)
        .and_then(|c| c[1].parse().ok())
        .unwrap_or(4096)
}

/// The line up to its first ';' OUTSIDE a quoted string.
fn strip_comment(raw: &str) -> &str {
    let mut q = false;
    for (i, c) in raw.char_indices() {
        if c == '"' {
            q = !q;
        } else if c == ';' && !q {
            return &raw[..i];
        }
    }
    raw
}

/// [OutputChannels] scalars -> {name: offset}.
fn och_channels(text: &str) -> HashMap<String, u16> {
    let re = Regex::new(r"^(\w+)\s*=\s*scalar\s*,\s*\w+\s*,\s*(\d+)").unwrap();
    let (mut out, mut on) = (HashMap::new(), false);
    for raw in text.lines() {
        let line = strip_comment(raw).trim();
        if line.starts_with('[') {
            on = line == "[OutputChannels]";
        } else if on {
            if let Some(c) = re.captures(line) {
                if let Ok(off) = c[2].parse() {
                    out.insert(c[1].to_string(), off);
                }
            }
        }
    }
    out
}

/// One field of the page: what the layout needs of a [Constants] line.
#[derive(Debug, Clone)]
struct Const {
    name: String,
    kind: String,
    typ: String,
    off: usize,
    shape: Vec<usize>,
    scale: f64,
    translate: f64,
    lo: Option<f64>,
    hi: Option<f64>,
    bits: Option<(u32, u32)>,
    labels: Option<Vec<String>>,
    width: usize,
    signed: bool,
}

impl Const {
    fn count(&self) -> usize {
        self.shape.iter().product()
    }
    fn same_encoding(&self, o: &Const) -> bool {
        self.typ == o.typ && self.scale == o.scale && self.translate == o.translate
    }
}

/// The fields in .ini order; a repeated name replaces the earlier one in place.
#[derive(Debug, Default)]
struct IniPage {
    fields: Vec<Const>,
    index: HashMap<String, usize>,
}

impl IniPage {
    fn get(&self, name: &str) -> Option<&Const> {
        self.index.get(name).map(|&i| &self.fields[i])
    }
}

fn num(t: &str) -> Option<f64> {
    let t = t.trim();
    if t.contains(['.', 'e', 'E']) {
        return t.parse().ok();
    }
    let (neg, body) = match t.strip_prefix('-') {
        Some(b) => (true, b),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    let v = if let Some(h) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
        i64::from_str_radix(h, 16).ok()?
    } else {
        body.parse::<i64>().ok()?
    } as f64;
    Some(if neg { -v } else { v })
}

fn split_outside_quotes(s: &str) -> Vec<String> {
    let (mut out, mut cur, mut q) = (Vec::new(), String::new(), false);
    for c in s.chars() {
        match c {
            '"' => {
                q = !q;
                cur.push(c);
            }
            ',' if !q => out.push(std::mem::take(&mut cur).trim().to_string()),
            _ => cur.push(c),
        }
    }
    out.push(cur.trim().to_string());
    out
}

fn parse_ini(text: &str) -> Result<IniPage, String> {
    let bits_re = Regex::new(r"^\[(\d+):(\d+)\]").unwrap();
    let mut page = IniPage::default();
    let mut on = false;
    for raw in text.lines() {
        let s = strip_comment(raw).trim();
        if s.starts_with('[') {
            on = s == "[Constants]";
            continue;
        }
        if !on {
            continue;
        }
        let Some((name, rhs)) = s.split_once('=') else {
            continue;
        };
        let name = name.trim().to_string();
        let parts = split_outside_quotes(rhs);
        let kind = parts[0].as_str();
        if !matches!(kind, "scalar" | "bits" | "array") {
            continue;
        }
        let bad = || format!("the .ini line for {name} cannot be read");
        let typ = parts.get(1).ok_or_else(bad)?.clone();
        let off: usize = parts.get(2).and_then(|p| p.parse().ok()).ok_or_else(bad)?;
        let (width, signed) = match typ.as_str() {
            "U08" => (1, false),
            "S08" => (1, true),
            "U16" => (2, false),
            "S16" => (2, true),
            "U32" => (4, false),
            "S32" => (4, true),
            _ => return Err(format!("{name}: type {typ} is not a page type")),
        };
        let mut c = Const {
            name: name.clone(),
            kind: kind.to_string(),
            typ,
            off,
            shape: Vec::new(),
            scale: 1.0,
            translate: 0.0,
            lo: None,
            hi: None,
            bits: None,
            labels: None,
            width,
            signed,
        };
        if kind == "bits" {
            let m = parts
                .get(3)
                .and_then(|p| bits_re.captures(p))
                .ok_or_else(bad)?;
            c.bits = Some((
                m[1].parse().map_err(|_| bad())?,
                m[2].parse().map_err(|_| bad())?,
            ));
            c.labels = Some(
                parts[4..]
                    .iter()
                    .map(|p| p.trim().trim_matches('"').to_string())
                    .collect(),
            );
        } else {
            let rest = if kind == "array" {
                let dims = parts.get(3).ok_or_else(bad)?;
                c.shape = dims
                    .trim_matches(['[', ']'])
                    .split('x')
                    .map(|x| x.trim().parse().map_err(|_| bad()))
                    .collect::<Result<_, _>>()?;
                &parts[4..]
            } else {
                &parts[3..]
            };
            let at = |i: usize| rest.get(i).and_then(|p| num(p));
            c.scale = at(1).unwrap_or(1.0);
            c.translate = at(2).unwrap_or(0.0);
            c.lo = at(3);
            c.hi = at(4);
        }
        match page.index.get(&name) {
            Some(&i) => page.fields[i] = c,
            None => {
                page.index.insert(name, page.fields.len());
                page.fields.push(c);
            }
        }
    }
    Ok(page)
}

/// The page's 2-D table storage order; no declaration = the old rpm-major.
fn table_order(text: &str) -> String {
    Regex::new(r"(?m)^;\s*tableOrder\s*=\s*([\w-]+)")
        .unwrap()
        .captures(text)
        .map(|c| c[1].to_string())
        .unwrap_or_else(|| "rpm-major".to_string())
}

// ------------------------------------------------------------ field migration

const NEVER: [&str; 3] = ["cfgVersion", "uMagic", "uSchema"];

fn label_equiv(old: &str) -> Option<&'static str> {
    match old {
        "Grounded" => Some("Active low (closed to ground = on)"),
        "High" => Some("Active high (open = on)"),
        _ => None,
    }
}

fn cavity(label: &str) -> Option<u32> {
    let digits: String = label.chars().take_while(char::is_ascii_digit).collect();
    match label[digits.len()..].chars().next() {
        Some(c) if c.is_alphanumeric() || c == '_' => None,
        _ => digits.parse().ok(),
    }
}

fn get_be(page: &[u8], off: usize, w: usize, signed: bool) -> i64 {
    let v = page[off..off + w]
        .iter()
        .fold(0i64, |a, &b| (a << 8) | b as i64);
    if signed && v >= 1 << (8 * w - 1) {
        v - (1 << (8 * w))
    } else {
        v
    }
}

fn put_be(page: &mut [u8], off: usize, w: usize, v: i64) {
    for i in 0..w {
        page[off + i] = (v >> (8 * (w - 1 - i))) as u8;
    }
}

type Report = BTreeMap<&'static str, Vec<String>>;

/// The saved page moved onto the new layout by field name: same encoding is
/// copied; a new scale is re-encoded and clamped; bits are mapped by label;
/// a new field keeps the firmware's default. Rules and reasons:
/// `me711_migrate_calibration.py`.
fn migrate(
    src_page: &[u8],
    src: &IniPage,
    base_page: &[u8],
    dst: &IniPage,
    src_order: &str,
    dst_order: &str,
) -> Result<(Vec<u8>, Report), String> {
    if dst.get("firingOrder").is_some() && src.get("firingOrder").is_none() {
        return Err("the ECU's page is older than any firmware with the K-line updater".into());
    }
    let mut page = base_page.to_vec();
    let mut rep: Report = [
        "copied",
        "converted",
        "clamped",
        "bits_moved",
        "by_label",
        "by_cavity",
        "transposed",
        "new_default",
        "dropped",
        "refused",
        "never",
    ]
    .into_iter()
    .map(|k| (k, Vec::new()))
    .collect();
    let mut push = |k: &'static str, v: String| rep.get_mut(k).expect("known key").push(v);
    let transpose = src_order != dst_order;
    let mut sorted: Vec<&Const> = dst.fields.iter().collect();
    sorted.sort_by_key(|c| c.off); // stable: .ini order within one offset

    for d in sorted {
        let name = d.name.clone();
        if NEVER.contains(&name.as_str()) {
            push("never", name);
            continue;
        }
        let Some(s) = src.get(&name) else {
            if name == "trigAngleX10" {
                if let Some(so) = src.get("trigOffsetX10") {
                    let v = get_be(src_page, so.off, so.width, so.signed) % 3600;
                    put_be(
                        &mut page,
                        d.off,
                        d.width,
                        if v > 1800 { v - 3600 } else { v },
                    );
                    push("converted", format!("{name} <- trigOffsetX10"));
                    continue;
                }
            }
            push("new_default", name);
            continue;
        };
        if s.kind != d.kind || s.shape != d.shape {
            push(
                "refused",
                format!("{name}: {}{:?} -> {}{:?}", s.kind, s.shape, d.kind, d.shape),
            );
            continue;
        }
        let bytes_s = s.off + s.width * s.count().max(1);
        let bytes_d = d.off + d.width * d.count().max(1);
        if bytes_s > src_page.len() || bytes_d > page.len() {
            push("refused", format!("{name}: outside the page"));
            continue;
        }
        if d.kind == "bits" {
            let ((lo_s, hi_s), (lo_d, hi_d)) = (s.bits.expect("bits"), d.bits.expect("bits"));
            let width_mask = |lo: u32, hi: u32| (1i64 << (hi - lo + 1)) - 1;
            let mut v = (get_be(src_page, s.off, s.width, false) >> lo_s) & width_mask(lo_s, hi_s);
            let (sl, dl) = (
                s.labels.as_ref().expect("bits"),
                d.labels.as_ref().expect("bits"),
            );
            if sl != dl {
                let at = v as usize;
                let lab = sl.get(at);
                let same_cavity = match (lab, dl.get(at)) {
                    (Some(a), Some(b)) => cavity(a).is_some() && cavity(a) == cavity(b),
                    _ => false,
                };
                let same_meaning = match (lab, dl.get(at)) {
                    (Some(a), Some(b)) => label_equiv(a) == Some(b.as_str()),
                    _ => false,
                };
                if same_cavity {
                    push("by_cavity", name.clone()); // the value IS the cavity
                } else if same_meaning {
                    push("by_label", name.clone()); // renamed, same meaning
                } else {
                    match lab
                        .filter(|l| l.as_str() != "INVALID")
                        .and_then(|l| dl.iter().position(|x| x == l))
                    {
                        Some(i) => {
                            v = i as i64;
                            push("by_label", name.clone());
                        }
                        None => {
                            push(
                                "refused",
                                format!("{name}: value {v} ({lab:?}) has no label in the new list"),
                            );
                            continue;
                        }
                    }
                }
            }
            if v >= 1 << (hi_d - lo_d + 1) {
                push(
                    "refused",
                    format!("{name}: value {v} does not fit [{lo_d}:{hi_d}]"),
                );
                continue;
            }
            let mask = width_mask(lo_d, hi_d) << lo_d;
            let cur = get_be(&page, d.off, d.width, false);
            put_be(&mut page, d.off, d.width, (cur & !mask) | (v << lo_d));
            if (lo_s, hi_s, s.off, &s.typ) != (lo_d, hi_d, d.off, &d.typ) {
                push("bits_moved", name);
            } else {
                push("copied", name);
            }
            continue;
        }
        let count = d.count().max(1);
        // element i of the destination comes from element order[i] of the source
        let mut order: Vec<usize> = (0..count).collect();
        if transpose && d.kind == "array" && d.shape.len() == 2 {
            let (nx, ny) = (d.shape[0], d.shape[1]); // [x x y]: index y*nx + x
            order = (0..ny)
                .flat_map(|y| (0..nx).map(move |x| x * ny + y))
                .collect();
            push("transposed", name.clone());
        }
        if s.same_encoding(d) {
            for (i, &j) in order.iter().enumerate() {
                let from = s.off + j * s.width;
                let chunk = src_page[from..from + s.width].to_vec();
                page[d.off + i * d.width..d.off + (i + 1) * d.width].copy_from_slice(&chunk);
            }
            push("copied", name);
            continue;
        }
        let mut clamped = false;
        for (i, &j) in order.iter().enumerate() {
            let raw = get_be(src_page, s.off + j * s.width, s.width, s.signed) as f64;
            let mut phys = (raw + s.translate) * s.scale;
            if let Some(lo) = d.lo {
                if phys < lo {
                    phys = lo;
                    clamped = true;
                }
            }
            if let Some(hi) = d.hi {
                if phys > hi {
                    phys = hi;
                    clamped = true;
                }
            }
            // half UP: a VE of one 0.5 % count must stay fuel, never become 0 %
            let new = (phys / d.scale - d.translate + 0.5).floor() as i64;
            put_be(&mut page, d.off + i * d.width, d.width, new);
        }
        push("converted", name.clone());
        if clamped {
            push("clamped", name);
        }
    }
    let dst_names: HashSet<&str> = dst.fields.iter().map(|c| c.name.as_str()).collect();
    let mut dropped: Vec<String> = src
        .fields
        .iter()
        .filter(|c| !dst_names.contains(c.name.as_str()) && c.name != "trigOffsetX10")
        .map(|c| c.name.clone())
        .collect();
    dropped.sort();
    rep.insert("dropped", dropped);
    Ok((page, rep))
}

// ------------------------------------------------------------- the whole job

/// What to update, and with what.
#[derive(Debug, Clone)]
pub struct UpdateRequest {
    pub image: PathBuf,
    pub port: String,
    /// The car's OBD socket (5-baud KWP entry) instead of the ECU's K-line pin.
    pub kwp: bool,
    /// The .ini of the firmware now in the ECU.
    pub old_ini: PathBuf,
    /// The .ini that came with the image.
    pub new_ini: PathBuf,
    /// Saved tune, log and result.
    pub out_dir: PathBuf,
}

/// Run one update on the real port. Every line of the log goes to `say`, and
/// to `<out_dir>/update.log`. The port must be free.
pub fn run_update(req: &UpdateRequest, say: &mut dyn FnMut(&str)) -> Result<String, String> {
    let bytes = std::fs::read(&req.image).map_err(|e| format!("{}: {e}", req.image.display()))?;
    let app = parse_app(&bytes).map_err(|e| format!("REFUSED: {e}"))?;
    let mut port = open_port(&req.port, Some(LINK_RATE)).map_err(|e| e.to_string())?;
    configure_port(port.as_mut()).map_err(|e| e.to_string())?;
    run_on_wire(port, &app, req, say)
}

fn run_on_wire<W: KWire>(
    wire: W,
    app: &AppImage,
    req: &UpdateRequest,
    say: &mut dyn FnMut(&str),
) -> Result<String, String> {
    std::fs::create_dir_all(&req.out_dir).map_err(|e| format!("{}: {e}", req.out_dir.display()))?;
    let log_path = req.out_dir.join("update.log");
    let mut log = String::new();
    let result = {
        let mut stamped = |m: &str| {
            let line = format!("{}  {m}", chrono::Local::now().format("%H:%M:%S%.3f"));
            say(&line);
            log.push_str(&line);
            log.push('\n');
            let _ = std::fs::write(&log_path, &log);
        };
        stamped(&format!(
            "image OK: board {}, build {}, {} bytes, crc 0x{:08X}, layout {}",
            app.board, app.build_id, app.length, app.crc, app.layout
        ));
        let r = update(wire, app, req, &mut stamped);
        if let Err(e) = &r {
            stamped(e_text(e));
        }
        r
    };
    result.map_err(String::from)
}

fn e_text(e: &E) -> &str {
    match e {
        E::Link(s) | E::Fatal(s) => s,
    }
}

fn update<W: KWire>(
    wire: W,
    app: &AppImage,
    req: &UpdateRequest,
    say: &mut dyn FnMut(&str),
) -> Result<String, E> {
    let out = &req.out_dir;
    let save = |name: &str, data: &[u8]| -> Result<(), E> {
        std::fs::write(out.join(name), data).map_err(|e| io_fatal(name, e))
    };
    let read_ini = |p: &Path| -> Result<String, E> {
        std::fs::read(p)
            .map(|b| String::from_utf8_lossy(&b).into_owned())
            .map_err(|e| E::Fatal(format!("{}: {e}", p.display())))
    };
    let mut k = Link::new(wire, LINK_RATE)?;
    if req.kwp {
        say("entering through the vehicle (KWP) path...");
        let rate = k.kwp_enter(LINK_RATE, say, 3)?;
        say(&format!("vehicle session at {rate} baud"));
    }
    let sig = match k.text(b"S") {
        Ok(s) => s,
        Err(_) => {
            // the ECU follows the host's rate on evidence (its link hunter);
            // give it a few frames at this rate, then fall back to 57600
            let mut found = None;
            for _ in 0..6 {
                if let Ok(s) = k.text(b"S") {
                    found = Some(s);
                    break;
                }
                sleep(ms(200));
            }
            match found {
                Some(s) => s,
                None => {
                    k.base_baud = 57600;
                    k.set_baud(57600)?;
                    let s = k.text(b"S")?;
                    say(&format!("link at 57600 (no answer at {LINK_RATE})"));
                    s
                }
            }
        }
    };
    let ver = k.text(b"V")?;
    let m = Regex::new(r"^(?:rusEFI )?HaniCore (\w+) ([0-9A-F]{8})$")
        .unwrap()
        .captures(&sig)
        .ok_or_else(|| E::Fatal(format!("not a HaniCore ECU: signature {sig:?}")))?;
    let (ecu_board, ecu_layout) = (m[1].to_string(), m[2].to_string());
    say(&format!("ECU   : {sig} | {ver}"));
    say(&format!("image : {} | {}", app.signature, app.version));
    if ecu_board != app.sig_board || board_text(&ver) != app.board_text {
        let is = board_text(&ver);
        return fatal(format!(
            "WRONG BOARD: the ECU is {:?}, the image is for {:?}",
            if is.is_empty() { &ecu_board } else { &is },
            app.board_text
        ));
    }
    let same_layout = ecu_layout == app.layout;
    say(&format!(
        "page layout {ecu_layout} -> {}: {}",
        app.layout,
        if same_layout {
            "SAME - the tune stays in the ECU"
        } else {
            "CHANGED - the tune will be migrated by field name"
        }
    ));
    let (old_text, new_text) = (read_ini(&req.old_ini)?, read_ini(&req.new_ini)?);
    let sig_line = |text: &str, s: &str| text.contains(s);
    if !sig_line(&new_text, &app.signature) {
        return fatal(format!(
            "{} is not the .ini for {:?} - keep the image's own .ini next to it",
            req.new_ini.display(),
            app.signature
        ));
    }
    if !sig_line(&old_text, &sig) {
        return fatal(format!(
            "the project's .ini ({}) is not the one for the ECU's firmware ({sig}): open the project that matches the ECU",
            req.old_ini.display()
        ));
    }
    let st = k.raw_status()?;
    if st.marker != UPD_MARKER {
        return fatal(format!(
            "the ECU's firmware has no K-line updater (OCH 490 = 0x{:04X}): flash an image with the updater once by bootmode",
            st.marker
        ));
    }
    let kwp = req.kwp;
    let mut u = Updater {
        k,
        say,
        kwp,
        fast: false,
        want_fast: false,
        erase_seq: None,
        erase_t0: Instant::now(),
        resends: 0,
    };
    u.begin_erase(st)?; // STAGING only; the tune is read meanwhile
    if u.k.base_baud >= LINK_RATE {
        u.go_fast()?;
    }
    // ---- 1. save the tune, always, before anything is written
    let page_before = u.k.page_read(ini_page_size(&old_text))?;
    save("page_before.bin", &page_before)?;
    let identity = serde_json::json!({
        "ecu_signature": sig, "ecu_version": ver, "ecu_ini": req.old_ini,
        "image": req.image, "image_signature": app.signature,
        "image_build": app.build_id, "image_ini": req.new_ini,
    });
    save(
        "identity.json",
        serde_json::to_string_pretty(&identity)
            .unwrap_or_default()
            .as_bytes(),
    )?;
    u.say(&format!(
        "tune saved: {}",
        out.join("page_before.bin").display()
    ));

    // ---- 2. stage, seal, commit
    u.stage(app, Instant::now())?;
    u.commit()?;
    u.say("COMMIT sent - the ECU resets into the loader");
    let back = u.wait_reboot(&app.build_id)?;
    u.say(&format!("ECU is back: {back}"));
    let sig2 = u.k.text(b"S")?;
    if sig2 != app.signature {
        return fatal(format!(
            "ECU signature {sig2:?} != image {:?}",
            app.signature
        ));
    }

    // ---- 3. keep the tune
    let base = u.k.page_read(ini_page_size(&new_text))?;
    save("page_after_update.bin", &base)?;
    let mut result = serde_json::json!({ "same_layout": same_layout, "new_build": app.build_id });
    let want = if same_layout {
        result["kept_without_writes"] = (base == page_before).into();
        u.say(&format!(
            "tune after the update {} the saved page",
            if base == page_before {
                "EQUALS"
            } else {
                "DIFFERS FROM"
            }
        ));
        page_before.clone()
    } else {
        let (want, rep) = migrate(
            &page_before,
            &parse_ini(&old_text).map_err(E::Fatal)?,
            &base,
            &parse_ini(&new_text).map_err(E::Fatal)?,
            &table_order(&old_text),
            &table_order(&new_text),
        )
        .map_err(E::Fatal)?;
        save(
            "migration_report.json",
            serde_json::to_string_pretty(&rep)
                .unwrap_or_default()
                .as_bytes(),
        )?;
        let n = |k: &str| rep.get(k).map_or(0, Vec::len);
        u.say(&format!(
            "migrated by field name: {} copied, {} converted, {} new (firmware default), {} dropped, {} refused",
            n("copied"),
            n("converted"),
            n("new_default"),
            n("dropped"),
            n("refused")
        ));
        want
    };
    save("page_restored.bin", &want)?;
    let (diff, burned) = u.restore_page(&want, &new_text, !same_layout)?;
    result["bytes_differing"] = diff.into();
    result["burned"] = burned.into();
    if burned {
        let ch = och_channels(&new_text);
        if let (Some(&a), Some(&b)) = (ch.get("nvmSource"), ch.get("nvmGeneration")) {
            let (src, generation) = (u.k.och16(a)?, u.k.och16(b)?);
            result["nvmSource"] = src.into();
            result["nvmGeneration"] = generation.into();
            u.say(&format!(
                "store: nvmSource 0x{src:04X}, generation {generation}"
            ));
        }
    }
    u.say(if burned {
        "tune restored and Burned"
    } else {
        "tune kept - nothing to write"
    });
    save(
        "result.json",
        serde_json::to_string_pretty(&result)
            .unwrap_or_default()
            .as_bytes(),
    )?;
    u.say(&format!("UPDATE COMPLETE: build {}", app.build_id));
    Ok(app.build_id.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    const OLD_INI: &str = r#"
; tableOrder = load-major
[Constants]
   page = 1
   pageSize = 64
   cfgVersion = scalar, U16, 0, "", 1, 0
   keep       = scalar, U16, 2, "rpm", 1, 0, 0, 9000, 0
   rescale    = scalar, U08, 4, "%", 0.5, 0, 0, 100, 1
   hot        = scalar, S16, 6, "C", 0.1, -1000, -40, 200, 1
   mode       = bits, U08, 8, [0:1], "Off", "Slow", "Fast", "INVALID"
   pin        = bits, U08, 9, [0:6], "0 none", "93 TMOT"
   gone       = scalar, U08, 10, "", 1, 0
   tab        = array, U08, 12, [2x3], "", 1, 0
[OutputChannels]
   cfgValidation = scalar, U16, 100, "", 1, 0
   nvmState      = scalar, U16, 102, "", 1, 0
"#;
    const NEW_INI: &str = r#"
; tableOrder = load-major
[Constants]
   page = 1
   pageSize = 80
   cfgVersion = scalar, U16, 0, "", 1, 0
   fresh      = scalar, U16, 2, "", 1, 0
   keep       = scalar, U16, 4, "rpm", 1, 0, 0, 9000, 0
   rescale    = scalar, U16, 6, "%", 0.1, 0, 0, 50, 1
   hot        = scalar, S16, 8, "C", 0.1, -1000, -40, 200, 1
   mode       = bits, U08, 10, [1:2], "Slow x", "Off", "Fast", "INVALID"
   pin        = bits, U08, 11, [0:6], "0 none", "93 - Coolant temperature"
   tab        = array, U08, 12, [2x3], "", 1, 0
[OutputChannels]
   cfgValidation = scalar, U16, 100, "", 1, 0
   nvmState      = scalar, U16, 102, "", 1, 0
   nvmSource     = scalar, U16, 104, "", 1, 0
   nvmGeneration = scalar, U16, 106, "", 1, 0
"#;

    fn image(sig: &str, ver_prefix: &str, build: u32, board: u16) -> Vec<u8> {
        let mut good = vec![0xFFu8; 0x100000];
        let h = HDR_OFF;
        good[h..h + 2].copy_from_slice(&HDR_MAGIC0.to_le_bytes());
        good[h + 2..h + 4].copy_from_slice(&HDR_MAGIC1.to_le_bytes());
        good[h + 4..h + 6].copy_from_slice(&board.to_le_bytes());
        let length = 0x1000u32;
        good[h + 10..h + 14].copy_from_slice(&length.to_le_bytes());
        good[h + 18..h + 22].copy_from_slice(&build.to_le_bytes());
        good[h + 30..h + 32].copy_from_slice(&HDR_SEAL.to_le_bytes());
        let ident = format!("{sig}\0{ver_prefix} build {build:08X}\0");
        good[APP_BASE + 0x400..APP_BASE + 0x400 + ident.len()].copy_from_slice(ident.as_bytes());
        for i in 0..0x300 {
            good[APP_BASE + 0x600 + i] = i as u8;
        }
        let mut c = crc32fast::Hasher::new();
        c.update(&good[APP_BASE..h]);
        c.update(&good[h + 32..APP_BASE + length as usize]);
        good[h + 14..h + 18].copy_from_slice(&c.finalize().to_le_bytes());
        good
    }

    fn app(layout: &str) -> AppImage {
        parse_app(&image(
            &format!("HaniCore ME75DC {layout}"),
            "HaniCore V1.2-dev / Bosch ME7.5 DC",
            0x12345678,
            4,
        ))
        .unwrap()
    }

    /// The ECU's side of the status contract behind a real frame layer: the
    /// cable echo, the envelope, every command. It models OUR protocol, not
    /// the flash - the flash path is proven on the bench.
    struct Fake {
        rx: VecDeque<u8>,
        pending: Vec<u8>,
        sig: String,
        ver: String,
        next_sig: String,
        next_ver: String,
        next_page: Option<Vec<u8>>,
        page: Vec<u8>,
        state: u16,
        err: u16,
        recv: u32,
        seq: u8,
        rc: u8,
        cmd: u8,
        crc: crc32fast::Hasher,
        erase_polls: u32,
        write_polls: u32,
        sealing: bool,
        mem: Vec<u8>,
        n: u32,
        wn: u32,
        writes: u32,
        burns: u32,
        page_writes: u32,
        drop_every: u32,
        lose_every: u32,
        busy_every: u32,
        refuse_begin: bool,
        marker: u16,
        validation: u16,
        log: std::sync::Arc<std::sync::Mutex<Vec<u32>>>,
    }

    impl Fake {
        fn new(layout_old: &str, app: &AppImage, page_len: usize) -> Fake {
            Fake {
                rx: VecDeque::new(),
                pending: Vec::new(),
                sig: format!("HaniCore ME75DC {layout_old}"),
                ver: "HaniCore V1.1 / Bosch ME7.5 DC build AAAAAAAA".into(),
                next_sig: app.signature.clone(),
                next_ver: app.version.clone(),
                next_page: None,
                page: (0..page_len).map(|i| (i * 7) as u8).collect(),
                state: ST_IDLE,
                err: 0,
                recv: 0,
                seq: 0,
                rc: 0,
                cmd: 0,
                crc: crc32fast::Hasher::new(),
                erase_polls: 0,
                write_polls: 0,
                sealing: false,
                mem: vec![0xFF; 0x40000],
                n: 0,
                wn: 0,
                writes: 0,
                burns: 0,
                page_writes: 0,
                drop_every: 0,
                lose_every: 0,
                busy_every: 0,
                refuse_begin: false,
                marker: UPD_MARKER,
                validation: 0,
                log: Default::default(),
            }
        }

        fn note(&mut self, c: u8, rc: u8) {
            self.cmd = c;
            self.rc = rc;
            self.seq = self.seq.wrapping_add(1);
        }

        fn status(&mut self) -> Vec<u8> {
            if self.state == ST_ERASING {
                self.erase_polls += 1;
                if self.erase_polls > 3 {
                    self.state = ST_READY;
                }
            } else if self.state == ST_WRITING {
                self.write_polls += 1;
                if self.write_polls > 1 {
                    self.state = if self.sealing { ST_SEALED } else { ST_READY };
                }
            }
            let w = [
                self.state,
                self.err,
                (self.recv >> 16) as u16,
                self.recv as u16,
                ((self.seq as u16) << 8) | self.rc as u16,
                self.cmd as u16,
                self.marker,
                0,
                4,
            ];
            w.iter().flat_map(|v| v.to_be_bytes()).collect()
        }

        fn handle(&mut self, p: &[u8]) {
            self.n += 1;
            if self.drop_every != 0 && self.n % self.drop_every == 0 {
                return; // the frame is lost
            }
            let mut data: Vec<u8> = Vec::new();
            let mut reply = true;
            match p[0] {
                b'S' => data = format!("{}\0", self.sig).into_bytes(),
                b'V' => data = format!("{}\0", self.ver).into_bytes(),
                b'O' => {
                    let off = u16::from_be_bytes([p[1], p[2]]);
                    let n = u16::from_be_bytes([p[3], p[4]]) as usize;
                    data = if off == UPD_OCH {
                        self.status()
                    } else if off == 100 {
                        self.validation.to_be_bytes().to_vec()
                    } else {
                        vec![0; n]
                    };
                }
                b'R' => {
                    let off = u16::from_be_bytes([p[1], p[2]]) as usize;
                    let n = u16::from_be_bytes([p[3], p[4]]) as usize;
                    data = self.page[off..off + n].to_vec();
                }
                b'W' => {
                    let off = u16::from_be_bytes([p[1], p[2]]) as usize;
                    self.page[off..off + p.len() - 3].copy_from_slice(&p[3..]);
                    self.page_writes += 1;
                }
                b'B' => self.burns += 1,
                b'!' => match p[1] {
                    b'u' => {
                        if self.refuse_begin {
                            self.note(b'u', 2);
                        } else {
                            self.state = ST_ERASING;
                            self.erase_polls = 0;
                            self.recv = 0;
                            self.crc = crc32fast::Hasher::new();
                            self.note(b'u', 0);
                        }
                    }
                    b'b' => self.note(b'b', 0),
                    b'w' => {
                        let off = u32::from_be_bytes([p[2], p[3], p[4], p[5]]);
                        let d = &p[6..];
                        self.wn += 1;
                        let busy = self.busy_every != 0 && self.wn % self.busy_every == 0;
                        if !(self.state == ST_READY || self.state == ST_WRITING)
                            || self.sealing
                            || off != self.recv
                            || busy
                        {
                            self.note(b'w', 3); // refused: out of order / full
                        } else {
                            self.writes += 1;
                            self.log.lock().unwrap().push(off);
                            self.mem[off as usize..off as usize + d.len()].copy_from_slice(d);
                            for (i, b) in d.iter().enumerate() {
                                let a = off as usize + i;
                                if !(0x200..0x220).contains(&a) {
                                    self.crc.update(&[*b]);
                                }
                            }
                            self.recv += d.len() as u32;
                            self.state = ST_WRITING;
                            self.write_polls = 0;
                            self.note(b'w', 0);
                        }
                    }
                    b'x' => {
                        let want = u32::from_be_bytes([p[2], p[3], p[4], p[5]]);
                        if self.crc.clone().finalize() != want {
                            self.state = ST_FAILED;
                            self.err = 3;
                            self.note(b'x', 3);
                        } else {
                            self.state = ST_WRITING;
                            self.write_polls = 0;
                            self.sealing = true;
                            self.note(b'x', 0);
                        }
                    }
                    b'y' => {
                        // COMMIT: resets, no reply; comes back as the new build
                        reply = false;
                        self.sig = self.next_sig.clone();
                        self.ver = self.next_ver.clone();
                        if let Some(pg) = self.next_page.take() {
                            self.page = pg;
                        }
                        self.state = ST_IDLE;
                        self.sealing = false;
                    }
                    _ => {}
                },
                _ => {}
            }
            if !reply || (self.lose_every != 0 && self.n % self.lose_every == 0) {
                return;
            }
            let mut blob = vec![0u8];
            blob.extend_from_slice(&data);
            let mut out = (blob.len() as u16).to_be_bytes().to_vec();
            out.extend_from_slice(&blob);
            out.extend_from_slice(&crc32fast::hash(&blob).to_be_bytes());
            self.rx.extend(out);
        }
    }

    impl KWire for Fake {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let n = buf.len().min(self.rx.len());
            if n == 0 {
                sleep(ms(1));
                return Err(io::Error::new(io::ErrorKind::TimedOut, "quiet"));
            }
            for b in buf.iter_mut().take(n) {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.rx.extend(buf.iter().copied()); // the cable's echo
            self.pending.extend_from_slice(buf);
            while self.pending.len() >= 2 {
                let n = u16::from_be_bytes([self.pending[0], self.pending[1]]) as usize;
                if self.pending.len() < n + 6 {
                    break;
                }
                let p = self.pending[2..2 + n].to_vec();
                self.pending.drain(..n + 6);
                self.handle(&p);
            }
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn set_timeout(&mut self, _t: Duration) -> io::Result<()> {
            Ok(())
        }
        fn set_baud(&mut self, _b: u32) -> io::Result<()> {
            Ok(())
        }
        fn set_break(&mut self, _on: bool) -> io::Result<()> {
            Ok(())
        }
        fn clear_input(&mut self) -> io::Result<()> {
            self.rx.clear();
            Ok(())
        }
        fn clear_output(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn bytes_to_read(&mut self) -> io::Result<u32> {
            Ok(self.rx.len() as u32)
        }
    }

    struct Dir(PathBuf);
    impl Dir {
        fn new(tag: &str, old: &str, new: &str) -> Dir {
            let d = std::env::temp_dir().join(format!(
                "hc_update_{tag}_{}_{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join("old.ini"), old).unwrap();
            std::fs::write(d.join("new.ini"), new).unwrap();
            Dir(d)
        }
        fn req(&self) -> UpdateRequest {
            UpdateRequest {
                image: self.0.join("x.bin"),
                port: "FAKE".into(),
                kwp: false,
                old_ini: self.0.join("old.ini"),
                new_ini: self.0.join("new.ini"),
                out_dir: self.0.join("out"),
            }
        }
    }
    impl Drop for Dir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// Same-layout update through the whole job: every chunk lands exactly
    /// once and in order, the tune stays, the files are written.
    fn same_layout(tag: &str, tweak: impl FnOnce(&mut Fake)) -> Result<(u32, Vec<String>), String> {
        let sig_ini = "signature = \"HaniCore ME75DC BBDC7743\"\n[Constants]\n   pageSize = 64\n[OutputChannels]\n";
        let d = Dir::new(tag, sig_ini, sig_ini);
        let a = app("BBDC7743");
        let mut f = Fake::new("BBDC7743", &a, 64);
        tweak(&mut f);
        let order = f.log.clone();
        let mut lines = Vec::new();
        let req = d.req();
        // the Fake is moved into the job: the write log above is its witness
        let r = run_on_wire(f, &a, &req, &mut |m| lines.push(m.to_string()));
        r?;
        let got = order.lock().unwrap().clone();
        for w in got.windows(2) {
            assert_eq!(w[1], w[0] + 64, "programmed out of order or twice: {got:?}");
        }
        assert_eq!(got.len(), plan_chunks(a.length as u32).len());
        let page = std::fs::read(req.out_dir.join("page_before.bin")).unwrap();
        assert_eq!(page.len(), 64);
        assert_eq!(
            std::fs::read(req.out_dir.join("page_restored.bin")).unwrap(),
            page
        );
        let res = std::fs::read_to_string(req.out_dir.join("result.json")).unwrap();
        assert!(res.contains("\"kept_without_writes\": true"), "{res}");
        assert!(req.out_dir.join("update.log").is_file());
        Ok((got.len() as u32, lines))
    }

    #[test]
    fn plan_covers_the_app_contiguously() {
        for len in [0x220u32, 0x221, 0x1000, 0x308A8, 0x3121E, 0x40000] {
            let c = plan_chunks(len);
            assert_eq!(c[0].0, 0);
            assert!(c[c.len() - 1].0 + c[c.len() - 1].1 >= len);
            for w in c.windows(2) {
                assert_eq!(w[1].0, w[0].0 + w[0].1);
            }
            assert!(c.iter().all(|&(o, n)| o % 2 == 0 && n % 2 == 0));
            assert!(c.iter().all(|&(o, n)| o >> 16 == (o + n - 1) >> 16));
        }
    }

    #[test]
    fn a_bad_image_is_refused() {
        let good = image("HaniCore ME71C4 83D8361E", "HaniCore V1 / X", 0x12345678, 2);
        let a = parse_app(&good).unwrap();
        assert_eq!(
            (a.sig_board.as_str(), a.layout.as_str()),
            ("ME71C4", "83D8361E")
        );
        assert_eq!(a.build_id, "12345678");
        assert_eq!(a.board_text, "X");
        let mut bad = good.clone();
        bad[HDR_OFF + 30] = 0;
        assert!(parse_app(&bad).unwrap_err().contains("sealed"));
        let mut bad = good.clone();
        bad[APP_BASE + 0x10] ^= 0xFF;
        assert!(parse_app(&bad).unwrap_err().contains("CRC"));
        assert!(parse_app(&good[..1000]).unwrap_err().contains("1 MiB"));
        let wrong_build = image("HaniCore ME71C4 83D8361E", "HaniCore V1 / X", 1, 2);
        let mut b = wrong_build;
        let at = APP_BASE + 0x400;
        let s = "HaniCore ME71C4 83D8361E\0HaniCore V1 / X build 00000002\0";
        b[at..at + s.len()].copy_from_slice(s.as_bytes());
        let mut c = crc32fast::Hasher::new();
        c.update(&b[APP_BASE..HDR_OFF]);
        c.update(&b[HDR_OFF + 32..APP_BASE + 0x1000]);
        b[HDR_OFF + 14..HDR_OFF + 18].copy_from_slice(&c.finalize().to_le_bytes());
        assert!(parse_app(&b).unwrap_err().contains("build"));
    }

    #[test]
    fn signature_with_the_old_prefix_is_still_recognised() {
        let a = parse_app(&image(
            "rusEFI HaniCore ME711 83D8361E",
            "HaniCore V1 / X",
            7,
            1,
        ))
        .unwrap();
        assert_eq!(a.signature, "rusEFI HaniCore ME711 83D8361E");
        assert_eq!(a.sig_board, "ME711");
    }

    #[test]
    fn clean_link_keeps_the_tune() {
        let (n, lines) = same_layout("clean", |_| {}).unwrap();
        assert_eq!(n, 64);
        assert!(lines
            .iter()
            .any(|l| l.contains("UPDATE COMPLETE: build 12345678")));
        assert!(lines.iter().any(|l| l.contains("EQUALS")));
    }

    #[test]
    fn lost_frames_are_resent_never_programmed_twice() {
        same_layout("drop", |f| f.drop_every = 7).unwrap();
    }

    #[test]
    fn lost_replies_do_not_program_a_chunk_twice() {
        same_layout("lose", |f| f.lose_every = 5).unwrap();
    }

    #[test]
    fn refused_chunks_resume_from_the_ecus_count() {
        let (_, lines) = same_layout("busy", |f| f.busy_every = 11).unwrap();
        assert!(lines.iter().any(|l| l.contains("resends")));
    }

    #[test]
    fn engine_running_stops_the_update() {
        let e = same_layout("refuse", |f| f.refuse_begin = true).unwrap_err();
        assert!(
            e.contains("REFUSED STAGE_BEGIN") && e.contains("VALIDATE"),
            "{e}"
        );
    }

    #[test]
    fn a_crc_the_ecu_does_not_match_refuses_the_seal() {
        let sig_ini = "signature = \"HaniCore ME75DC BBDC7743\"\n[Constants]\n   pageSize = 64\n";
        let d = Dir::new("crc", sig_ini, sig_ini);
        let mut a = app("BBDC7743");
        let f = Fake::new("BBDC7743", &a, 64);
        a.crc ^= 1;
        let e = run_on_wire(f, &a, &d.req(), &mut |_| {}).unwrap_err();
        assert!(e.contains("SEAL"), "{e}");
    }

    #[test]
    fn wrong_board_and_no_updater_are_refused_before_anything_moves() {
        let sig_ini = "signature = \"HaniCore ME75DC BBDC7743\"\n[Constants]\n   pageSize = 64\n";
        let d = Dir::new("board", sig_ini, sig_ini);
        let a = app("BBDC7743");
        let mut f = Fake::new("BBDC7743", &a, 64);
        f.ver = "HaniCore V1.1 / Bosch ME7.5 BFB build AAAAAAAA".into();
        let e = run_on_wire(f, &a, &d.req(), &mut |_| {}).unwrap_err();
        assert!(e.contains("WRONG BOARD"), "{e}");
        let mut f = Fake::new("BBDC7743", &a, 64);
        f.marker = 0;
        let e = run_on_wire(f, &a, &d.req(), &mut |_| {}).unwrap_err();
        assert!(e.contains("no K-line updater"), "{e}");
    }

    #[test]
    fn the_project_ini_must_match_the_ecu() {
        let right = "signature = \"HaniCore ME75DC BBDC7743\"\n[Constants]\n   pageSize = 64\n";
        let wrong = "signature = \"HaniCore ME75DC 00000000\"\n[Constants]\n   pageSize = 64\n";
        let d = Dir::new("ini", wrong, right);
        let a = app("BBDC7743");
        let f = Fake::new("BBDC7743", &a, 64);
        let e = run_on_wire(f, &a, &d.req(), &mut |_| {}).unwrap_err();
        assert!(e.contains("not the one for the ECU"), "{e}");
    }

    fn old_page() -> Vec<u8> {
        let mut p = vec![0u8; 64];
        p[0..2].copy_from_slice(&0xBEEFu16.to_be_bytes()); // cfgVersion
        p[2..4].copy_from_slice(&6500u16.to_be_bytes()); // keep
        p[4] = 40; // rescale: 20.0 %
        p[6..8].copy_from_slice(&1900i16.to_be_bytes()); // hot: 90.0 C
        p[8] = 2; // mode: Fast
        p[9] = 1; // pin: "93 TMOT"
        for i in 0..6 {
            p[12 + i] = 100 + i as u8; // tab
        }
        p
    }

    #[test]
    fn migration_moves_fields_by_name() {
        let (src, dst) = (parse_ini(OLD_INI).unwrap(), parse_ini(NEW_INI).unwrap());
        let base = vec![0x11u8; 80];
        let (p, rep) = migrate(&old_page(), &src, &base, &dst, "load-major", "load-major").unwrap();
        assert_eq!(
            &p[0..2],
            &[0x11, 0x11],
            "cfgVersion is the page's own format"
        );
        assert_eq!(
            &p[2..4],
            &[0x11, 0x11],
            "a new field keeps the firmware default"
        );
        assert_eq!(
            u16::from_be_bytes([p[4], p[5]]),
            6500,
            "same encoding follows its name"
        );
        assert_eq!(
            u16::from_be_bytes([p[6], p[7]]),
            200,
            "20.0 % re-encoded at 0.1"
        );
        assert_eq!(i16::from_be_bytes([p[8], p[9]]), 1900, "signed + translate");
        assert_eq!((p[10] >> 1) & 3, 2, "bits by label: Fast");
        assert_eq!(
            p[10] & 1,
            1,
            "the other bits of that byte stay the firmware's"
        );
        assert_eq!(p[11], 1, "the value IS the cavity");
        assert_eq!(&p[12..18], &old_page()[12..18]);
        assert_eq!(rep["new_default"], ["fresh"]);
        assert_eq!(rep["dropped"], ["gone"]);
        assert_eq!(rep["never"], ["cfgVersion"]);
        assert!(rep["by_cavity"].contains(&"pin".to_string()));
        assert!(rep["bits_moved"].contains(&"mode".to_string()));
        assert!(rep["refused"].is_empty(), "{:?}", rep["refused"]);
    }

    #[test]
    fn migration_clamps_rounds_transposes_and_refuses() {
        let (src, dst) = (parse_ini(OLD_INI).unwrap(), parse_ini(NEW_INI).unwrap());
        let base = vec![0u8; 80];
        let mut sp = old_page();
        sp[4] = 200; // rescale: 100.0 %, new maximum 50
        sp[8] = 3; // mode: INVALID is never mapped
        let (p, rep) = migrate(&sp, &src, &base, &dst, "rpm-major", "load-major").unwrap();
        assert_eq!(u16::from_be_bytes([p[6], p[7]]), 500);
        assert!(rep["clamped"].contains(&"rescale".to_string()));
        assert_eq!((p[10] >> 1) & 3, 0, "refused: the firmware's value stays");
        assert!(rep["refused"].iter().any(|r| r.starts_with("mode")));
        // [2x3]: element (x, y) comes from source index x*ny + y
        assert_eq!(&p[12..18], &[100, 103, 101, 104, 102, 105]);
        assert_eq!(rep["transposed"], ["tab"]);
        // half UP, never to even
        let one = "[Constants]\n v = scalar, U08, 0, \"\", 0.5, 0\n";
        let whole = "[Constants]\n v = scalar, U08, 0, \"\", 1, 0\n";
        let (q, _) = migrate(
            &[1, 0],
            &parse_ini(one).unwrap(),
            &[9, 0],
            &parse_ini(whole).unwrap(),
            "a",
            "a",
        )
        .unwrap();
        assert_eq!(q[0], 1, "0.5 rounds up");
        // a page from before firingOrder is older than the updater
        let new = "[Constants]\n firingOrder = scalar, U08, 0, \"\", 1, 0\n";
        assert!(migrate(
            &[0],
            &parse_ini(whole).unwrap(),
            &[0],
            &parse_ini(new).unwrap(),
            "a",
            "a"
        )
        .is_err());
    }

    #[test]
    fn a_layout_change_migrates_writes_and_burns() {
        let old = format!("signature = \"HaniCore ME75DC 11111111\"\n{OLD_INI}");
        let new = format!("signature = \"HaniCore ME75DC 22222222\"\n{NEW_INI}");
        let d = Dir::new("layout", &old, &new);
        let a = app("22222222");
        let mut f = Fake::new("11111111", &a, 64);
        f.page = old_page();
        f.next_page = Some(vec![0x11; 80]);
        let req = d.req();
        let mut lines = Vec::new();
        run_on_wire(f, &a, &req, &mut |m| lines.push(m.to_string())).unwrap();
        assert!(lines.iter().any(|l| l.contains("CHANGED")));
        assert!(lines.iter().any(|l| l.contains("restored and Burned")));
        assert!(lines.iter().any(|l| l.contains("generation")));
        let want = std::fs::read(req.out_dir.join("page_restored.bin")).unwrap();
        assert_eq!(u16::from_be_bytes([want[4], want[5]]), 6500);
        let res = std::fs::read_to_string(req.out_dir.join("result.json")).unwrap();
        assert!(res.contains("\"burned\": true"), "{res}");
        assert!(req.out_dir.join("migration_report.json").is_file());
    }

    #[test]
    fn a_page_the_firmware_refuses_is_not_burned() {
        let old = format!("signature = \"HaniCore ME75DC 11111111\"\n{OLD_INI}");
        let new = format!("signature = \"HaniCore ME75DC 22222222\"\n{NEW_INI}");
        let d = Dir::new("noburn", &old, &new);
        let a = app("22222222");
        let mut f = Fake::new("11111111", &a, 64);
        f.page = old_page();
        f.next_page = Some(vec![0x11; 80]);
        f.validation = 0x0040;
        let req = d.req();
        let e = run_on_wire(f, &a, &req, &mut |_| {}).unwrap_err();
        assert!(e.contains("NOT burning"), "{e}");
        assert!(
            req.out_dir.join("page_before.bin").is_file(),
            "the tune is on disk"
        );
    }

    /// Differential against the Python original on real files:
    /// HC_MIG_OLD_INI, HC_MIG_NEW_INI, HC_MIG_OLD_PAGE, HC_MIG_BASE_PAGE,
    /// HC_MIG_WANT (the page Python produced), HC_IMAGE.
    #[test]
    #[ignore = "needs real HaniCore files, see the doc comment"]
    fn real_files_match_the_python_original() {
        let var = |n: &str| std::env::var(n).unwrap_or_else(|_| panic!("{n} not set"));
        let text = |n: &str| String::from_utf8_lossy(&std::fs::read(var(n)).unwrap()).into_owned();
        let (old, new) = (text("HC_MIG_OLD_INI"), text("HC_MIG_NEW_INI"));
        let (page, _rep) = migrate(
            &std::fs::read(var("HC_MIG_OLD_PAGE")).unwrap(),
            &parse_ini(&old).unwrap(),
            &std::fs::read(var("HC_MIG_BASE_PAGE")).unwrap(),
            &parse_ini(&new).unwrap(),
            &table_order(&old),
            &table_order(&new),
        )
        .unwrap();
        assert_eq!(page, std::fs::read(var("HC_MIG_WANT")).unwrap());
    }

    /// HC_IMAGE: what the Python original must also read out of that image.
    #[test]
    #[ignore = "needs a real HaniCore image"]
    fn real_image_parse() {
        let a = parse_app(&std::fs::read(std::env::var("HC_IMAGE").unwrap()).unwrap()).unwrap();
        println!(
            "{}|{}|{}|0x{:08X}|{}|{:08X}",
            a.signature,
            a.version,
            a.build_id,
            a.crc,
            a.length,
            crc32fast::hash(&a.body)
        );
    }
}
