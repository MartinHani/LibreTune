// Modified 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
// Added 2026-10-30 in the HaniCore K-line fork of LibreTune (see FORK.md);
// GPL-2.0, as the rest of LibreTune.
//! K-line transport for an ordinary KKL (FTDI K-line) cable.
//!
//! A K-line is one wire for both directions, so the cable hands back every
//! byte the host sends before the ECU's reply. [`KLineChannel`] remembers
//! what it wrote and drops exactly those bytes from the front of the read
//! stream, so the protocol above sees only the ECU. The echo is matched by
//! content: a byte that differs (an echo the adapter lost) gives the echo up,
//! and that byte belongs to the ECU. On top of that it can find the ECU:
//!
//! * [`KLineMode::EchoOnly`] — the echo filter alone (`filterEchoBytes = true`
//!   in the INI on a plain serial connection).
//! * [`KLineMode::Direct`] — the cable on the ECU's K-line pin. The ECU may
//!   hunt between several baud rates and locks onto the first CRC-valid
//!   frame, so a signature request (`S`) is repeated until it answers at the
//!   chosen rate.
//! * [`KLineMode::Kwp`] — the car's OBD socket (VAG/KWP2000 style):
//!   5-baud address 0x01, the ISO 14230 key-byte handshake, StartCommunication,
//!   an IDENTIFY and a GOFAST that moves the line to the session rate.
//!
//! In both entry modes a supervisor thread keeps the session alive (an `S`
//! request when the line has been quiet for 0.8 s: the ECU lets the
//! negotiated rate go after 1.5 s without a valid request) and enters again
//! when a request goes unanswered (2.5 s; 9 s for `B`urn and `!` commands,
//! whose replies are deferred), e.g. after the ECU was power-cycled.
//!
//! The ECU side of this protocol is HaniCore's (Bosch ME7 standalone
//! firmware); any ECU that speaks the TunerStudio CRC envelope over a K-line
//! with the same entry can use it.

use serialport::SerialPort;
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::{Duration, Instant};

use super::stream::CommunicationChannel;
use super::ProtocolError;

/// How the K-line channel finds and holds the ECU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KLineMode {
    /// Echo filter only; no entry, no supervision.
    EchoOnly,
    /// Cable on the ECU's K-line pin: rate hunt, keepalive, re-entry.
    Direct,
    /// Car's OBD socket: 5-baud + KWP entry, GOFAST, keepalive, re-entry.
    Kwp,
}

/// An echo that has not come back in this long is given up.
const ECHO_LOST: Duration = Duration::from_millis(300);
/// Quiet time after which a keepalive request is sent.
const KEEPALIVE: Duration = Duration::from_millis(800);
/// A request unanswered this long means the ECU is gone.
const REPLY: Duration = Duration::from_millis(2500);
/// Same for requests with deferred replies (`B`urn, `!` commands).
const REPLY_SLOW: Duration = Duration::from_millis(9000);
/// Entry attempts per (re-)entry.
const ENTRY_TRIES: usize = 3;
/// Direct path: how long to keep asking for the signature.
const FIND_LINK: Duration = Duration::from_secs(10);
/// KWP: one bit of the 5-baud address.
const FIVE_BAUD_BIT: Duration = Duration::from_millis(200);
/// The HaniCore TunerStudio plugin's ReplyWatch (2.3.0, what made OBD stable on
/// the Porsche): the ECU answers within ~30 ms, so a request with no reply byte
/// after this long never reached it and is sent again - twice at most.
const RESEND: Duration = Duration::from_millis(300);
const RESENDS: u8 = 2;
/// KWP: idle before the 5-baud address (ISO W5, the plugin's idle(300)).
const W5: Duration = Duration::from_millis(300);
/// OBD: the line left alone before a re-entry. A 5-baud sent right after traffic
/// failed nearly every time in the car (2026-11-05), after this much silence it
/// worked.
const QUIET: Duration = Duration::from_millis(2500);
/// OBD: how long the DIRECT way in is tried before the 5-baud one. The plugin
/// 2.3.0 that held OBD on the Porsche connected with AUTO = remembered path, then
/// DIRECT (a TunerStudio request at the chosen rate, 8 tries), then 5-baud; its
/// memory on the car laptop was DIRECT, so OBD ran without any KWP entry, while
/// every session entered by 5-baud died (2026-11-05 review). The ECU answers a
/// CRC-valid request at any time and follows the host's rate by itself.
const DIRECT_FIRST: Duration = Duration::from_millis(3000);

/// A read or a value write repeats safely; a Burn, a `!` action or a logger read
/// does not (the plugin's list).
fn resendable(cmd: u8) -> bool {
    matches!(
        cmd,
        b'S' | b'Q' | b'V' | b'F' | b'C' | b'I' | b'f' | b'O' | b'A' | b'P' | b'R' | b'p' | b'W'
    )
}

/// KWP GOFAST rate codes (HaniCore `B1 <code>`).
fn rate_code(rate: u32) -> Option<u8> {
    match rate {
        10400 => Some(0x01),
        57600 => Some(0x02),
        124800 => Some(0x03),
        115200 => Some(0x04),
        _ => None,
    }
}

/// The few serial operations the K-line needs; implemented for a real port
/// and, in the tests, for a simulated cable + ECU.
pub trait KWire: Send {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write(&mut self, buf: &[u8]) -> io::Result<usize>;
    fn flush(&mut self) -> io::Result<()>;
    fn set_timeout(&mut self, t: Duration) -> io::Result<()>;
    fn set_baud(&mut self, baud: u32) -> io::Result<()>;
    fn set_break(&mut self, on: bool) -> io::Result<()>;
    fn clear_input(&mut self) -> io::Result<()>;
    fn clear_output(&mut self) -> io::Result<()>;
    fn bytes_to_read(&mut self) -> io::Result<u32>;
}

impl KWire for Box<dyn SerialPort> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        Read::read(self, buf)
    }
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        Write::write(self, buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        Write::flush(self)
    }
    fn set_timeout(&mut self, t: Duration) -> io::Result<()> {
        SerialPort::set_timeout(self.as_mut(), t).map_err(io::Error::other)
    }
    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.set_baud_rate(baud).map_err(io::Error::other)
    }
    fn set_break(&mut self, on: bool) -> io::Result<()> {
        if on {
            SerialPort::set_break(self.as_ref())
        } else {
            self.clear_break()
        }
        .map_err(io::Error::other)
    }
    fn clear_input(&mut self) -> io::Result<()> {
        self.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)
    }
    fn clear_output(&mut self) -> io::Result<()> {
        self.clear(serialport::ClearBuffer::Output)
            .map_err(io::Error::other)
    }
    fn bytes_to_read(&mut self) -> io::Result<u32> {
        SerialPort::bytes_to_read(self.as_ref()).map_err(io::Error::other)
    }
}

/// TunerStudio CRC envelope: big-endian length, payload, CRC-32.
pub(crate) fn envelope(payload: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(payload.len() + 6);
    v.extend_from_slice(&(payload.len() as u16).to_be_bytes());
    v.extend_from_slice(payload);
    v.extend_from_slice(&crc32fast::hash(payload).to_be_bytes());
    v
}

/// KWP frame with a format byte, target 0x01, source 0xF1 and a checksum.
fn kwp_frame(data: &[u8]) -> Vec<u8> {
    let mut f = vec![0x80 | data.len() as u8, 0x01, 0xF1];
    f.extend_from_slice(data);
    let sum = f.iter().fold(0u8, |a, b| a.wrapping_add(*b));
    f.push(sum);
    f
}

struct Inner<W: KWire> {
    wire: W,
    mode: KLineMode,
    /// Requested rate; after a KWP entry, the negotiated one.
    rate: u32,
    /// Bytes written whose echo has not come back yet.
    echo: VecDeque<u8>,
    /// ECU bytes read from the wire, not yet handed to the caller.
    rxq: VecDeque<u8>,
    /// Bytes read against the current echo so far, and how many agreed.
    matched: Vec<u8>,
    same: usize,
    /// The request being assembled from the caller's writes.
    frame: Vec<u8>,
    /// A complete request with no reply byte yet, when it went out, re-sends so far.
    armed: Option<Vec<u8>>,
    t_armed: Instant,
    resends: u8,
    /// When the echo being waited for went out (a re-send is later than t_tx).
    t_echo: Instant,
    /// Last write / last ECU byte (echo excluded).
    t_tx: Instant,
    t_rx: Instant,
    /// How long the last request may take.
    reply_limit: Duration,
    /// Caller's read timeout.
    timeout: Duration,
    five_baud_bit: Duration,
}

impl<W: KWire> Inner<W> {
    /// Read up to `n` bytes within `limit` (bypasses the echo filter).
    fn read_raw(&mut self, n: usize, limit: Duration) -> Vec<u8> {
        let mut got = vec![0u8; n];
        let mut have = 0;
        let end = Instant::now() + limit;
        let _ = self.wire.set_timeout(Duration::from_millis(20));
        while have < n && Instant::now() < end {
            match self.wire.read(&mut got[have..]) {
                Ok(k) => have += k,
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(_) => break,
            }
        }
        let _ = self.wire.set_timeout(self.timeout);
        got.truncate(have);
        got
    }

    /// Write and take the cable's echo of it back off the line.
    fn send_raw(&mut self, data: &[u8]) -> bool {
        if self.wire.write(data).is_err() || self.wire.flush().is_err() {
            return false;
        }
        self.read_raw(data.len(), Duration::from_millis(500)) == data
    }

    /// One enveloped request, the reply's payload if it is CRC-valid.
    fn ts_request(&mut self, payload: &[u8], limit: Duration) -> Option<Vec<u8>> {
        let req = envelope(payload);
        let _ = self.wire.clear_input();
        if !self.send_raw(&req) {
            return None;
        }
        let head = self.read_raw(2, limit);
        if head.len() < 2 {
            return None;
        }
        let n = u16::from_be_bytes([head[0], head[1]]) as usize;
        if n == 0 || n > 0x800 {
            return None;
        }
        let rest = self.read_raw(n + 4, limit);
        if rest.len() < n + 4 {
            return None;
        }
        let body = &rest[..n];
        let crc = u32::from_be_bytes([rest[n], rest[n + 1], rest[n + 2], rest[n + 3]]);
        (crc == crc32fast::hash(body)).then(|| body.to_vec())
    }

    fn kwp_request(&mut self, data: &[u8], limit: Duration) -> Option<Vec<u8>> {
        if !self.send_raw(&kwp_frame(data)) {
            return None;
        }
        let mut head = self.read_raw(3, limit);
        if head.len() < 3 {
            return None;
        }
        let mut n = (head[0] & 0x3F) as usize;
        let mut extra = 0;
        if n == 0 {
            let ln = self.read_raw(1, limit);
            if ln.is_empty() {
                return None;
            }
            n = ln[0] as usize;
            extra = 1;
            head.extend_from_slice(&ln);
        }
        let rest = self.read_raw(n + 1, limit);
        if rest.len() < n + 1 {
            return None;
        }
        let mut frame = head;
        frame.extend_from_slice(&rest);
        let last = frame.len() - 1;
        let sum = frame[..last].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if sum != frame[last] {
            return None;
        }
        Some(frame[3 + extra..3 + extra + n].to_vec())
    }

    /// 5-baud address byte by the adapter's BREAK, LSB first.
    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        self.wire.set_baud(10400)?;
        let t0 = Instant::now();
        for i in 0..10u32 {
            let bit = match i {
                0 => 0,
                9 => 1,
                _ => (addr >> (i - 1)) & 1,
            };
            self.wire.set_break(bit == 0)?;
            let until = t0 + self.five_baud_bit * (i + 1);
            let now = Instant::now();
            if until > now {
                std::thread::sleep(until - now);
            }
        }
        self.wire.set_break(false)?;
        self.wire.clear_input()
    }

    /// One KWP entry; the negotiated rate on success.
    fn kwp_attempt(&mut self) -> Result<u32, String> {
        let _ = self.wire.set_baud(10400);
        std::thread::sleep(W5.min(self.five_baud_bit * 2));
        self.five_baud(0x01).map_err(|e| e.to_string())?;
        let sync = self.read_raw(3, Duration::from_millis(400));
        if sync != [0x55, 0xEF, 0x8F] {
            return Err(format!("no 5-baud sync ({:02X?})", sync));
        }
        std::thread::sleep(Duration::from_millis(25)); // W4: 25..50 ms, a Windows sleep runs long
        self.send_raw(&[0x70]);
        let fe = self.read_raw(1, Duration::from_millis(200));
        if fe != [0xFE] {
            return Err(format!("no key-byte complement ({:02X?})", fe));
        }
        match self.kwp_request(&[0x81], Duration::from_millis(300)) {
            Some(d) if d.first() == Some(&0xC1) => {}
            d => return Err(format!("StartCommunication: {:02X?}", d)),
        }
        match self.kwp_request(&[0xB0, b'H', b'C'], Duration::from_millis(600)) {
            Some(d) if d.first() == Some(&0xF0) => {
                tracing::info!("K-line: ECU {}", String::from_utf8_lossy(&d[1..]));
            }
            d => return Err(format!("IDENTIFY: {:02X?}", d)),
        }
        // 10400 too (2026-11-06): the session then stays at the entry's own
        // rate, as VCDS does on the AWT car
        let mut rate = if rate_code(self.rate).is_some() {
            self.rate
        } else {
            124800
        };
        let mut d = self.kwp_request(
            &[0xB1, rate_code(rate).unwrap()],
            Duration::from_millis(300),
        );
        // a refused rate (7F B1 ..: the ECU cannot make it) - same session, the
        // next one: 124800 first, the nearest to the 115200 OBD ran at on the Porsche
        for alt in [124800u32, 57600] {
            if alt == rate || !matches!(&d, Some(v) if v.starts_with(&[0x7F, 0xB1])) {
                continue;
            }
            tracing::info!("K-line: GOFAST {} refused, trying {}", rate, alt);
            rate = alt;
            d = self.kwp_request(
                &[0xB1, rate_code(rate).unwrap()],
                Duration::from_millis(300),
            );
        }
        match d {
            Some(v) if v.starts_with(&[0xF1, rate_code(rate).unwrap()]) => {}
            d => return Err(format!("GOFAST: {:02X?}", d)),
        }
        std::thread::sleep(Duration::from_millis(20)); // the ECU switches after its reply
        self.wire.set_baud(rate).map_err(|e| e.to_string())?;
        let _ = self.wire.clear_input();
        match self.ts_request(b"V", Duration::from_millis(500)) {
            Some(b) if b.first() == Some(&0) => Ok(rate),
            b => Err(format!("no answer at {} baud ({:02X?})", rate, b)),
        }
    }

    /// Direct path: ask for the signature until the ECU answers at our rate.
    fn find_link(&mut self, limit: Duration) -> bool {
        let _ = self.wire.set_baud(self.rate);
        let end = Instant::now() + limit;
        while Instant::now() < end {
            if let Some(b) = self.ts_request(b"S", Duration::from_millis(300)) {
                if b.first() == Some(&0) {
                    tracing::info!(
                        "K-line: ECU at {} baud: {}",
                        self.rate,
                        String::from_utf8_lossy(&b[1..])
                    );
                    return true;
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        false
    }

    /// OBD, the fallback: the DIRECT way in, at the chosen rate and then at
    /// every board's boot rate (57600). -> true when the ECU answered.
    fn direct_first(&mut self) -> bool {
        let asked = self.rate;
        let limit = DIRECT_FIRST.min(self.five_baud_bit * 15);
        for rate in [asked, 57600] {
            if rate != asked && asked == 57600 {
                break;
            }
            self.rate = rate;
            if self.find_link(limit) {
                tracing::info!("K-line: OBD path DIRECT at {} baud (no 5-baud sent)", rate);
                return true;
            }
        }
        self.rate = asked;
        tracing::info!("K-line: OBD path: no direct answer either");
        false
    }

    /// (Re-)enter: find the ECU and leave the line clean.
    fn enter(&mut self) -> Result<(), String> {
        let mode = self.mode;
        let r = match mode {
            KLineMode::EchoOnly => Ok(()),
            KLineMode::Direct => {
                let asked = self.rate;
                if self.find_link(FIND_LINK) {
                    Ok(())
                } else if asked != 57600 && {
                    // Every HaniCore board boots its link at 57600; not every
                    // board can make the other rates (the 24 MHz ones have no
                    // 115200), so a rate it cannot make is not a dead ECU.
                    self.rate = 57600;
                    self.find_link(FIND_LINK)
                } {
                    tracing::warn!("K-line: no answer at {} baud, connected at 57600", asked);
                    Ok(())
                } else {
                    self.rate = asked;
                    Err(format!(
                        "no answer at {} baud - ECU powered? cable on the K-line pin?",
                        asked
                    ))
                }
            }
            // DIRECT FIRST, THE PLUGIN'S ORDER (2026-11-06 night, again): plugin
            // 2.3.0's AUTO tried DIRECT before the 5-baud entry and never sent a
            // 5-baud once DIRECT had answered - and that held OBD on the AWT car.
            // Every car session that died (Porsche 15.10, 018DC 04..06.11, all
            // three tools) had been entered by 5-baud. 10400 has no direct path
            // (no board hunts to it): the vehicle entry only.
            KLineMode::Kwp if self.rate != 10400 && self.direct_first() => Ok(()),
            KLineMode::Kwp => {
                let asked = self.rate;
                let mut last = String::new();
                let mut ok = None;
                for i in 0..ENTRY_TRIES {
                    match self.kwp_attempt() {
                        Ok(rate) => {
                            ok = Some(rate);
                            break;
                        }
                        Err(e) => {
                            tracing::warn!("K-line: KWP entry {} failed: {}", i + 1, e);
                            last = e;
                            // leave the line alone before the next address
                            std::thread::sleep(QUIET.min(self.five_baud_bit * 13));
                        }
                    }
                }
                match ok {
                    Some(rate) => {
                        tracing::info!("K-line: vehicle session at {} baud", rate);
                        self.rate = rate;
                        Ok(())
                    }
                    None => {
                        self.rate = asked;
                        if asked == 10400 && self.direct_first() {
                            Ok(())
                        } else {
                            Err(format!(
                                "no KWP entry ({}), no direct answer - ignition on? cable in the OBD socket?",
                                last
                            ))
                        }
                    }
                }
            }
        };
        let _ = self.wire.clear_input();
        self.forget();
        r
    }

    /// Nothing outstanding on the line any more.
    fn forget(&mut self) {
        self.echo.clear();
        self.matched.clear();
        self.same = 0;
        self.rxq.clear();
        self.frame.clear();
        self.armed = None;
        let now = Instant::now();
        self.t_tx = now;
        self.t_rx = now;
        self.t_echo = now;
    }

    /// The caller wrote these bytes: a complete envelope that may be repeated is
    /// armed (the plugin assembles it the same way - it can come in pieces).
    fn wrote(&mut self, buf: &[u8]) {
        self.armed = None;
        self.frame.extend_from_slice(buf);
        if self.frame.len() < 2 {
            return;
        }
        let n = u16::from_be_bytes([self.frame[0], self.frame[1]]) as usize;
        if n == 0 || self.frame.len() > n + 6 {
            self.frame.clear();
            return;
        }
        if self.frame.len() < n + 6 {
            return;
        }
        let f = std::mem::take(&mut self.frame);
        if resendable(f[2]) {
            self.armed = Some(f);
            self.t_armed = Instant::now();
            self.resends = 0;
        }
    }

    /// No reply byte for RESEND: the request never reached the ECU, send it again.
    /// -> true while a request is still being repeated.
    fn resend_if_due(&mut self) -> bool {
        if self.armed.is_none() {
            return false;
        }
        if let Ok(n) = self.wire.bytes_to_read() {
            if n > 0 {
                let got = self.read_raw(n as usize, Duration::from_millis(5));
                self.take(&got);
            }
        }
        if self.armed.is_none() || self.t_armed.elapsed() < RESEND {
            return self.armed.is_some();
        }
        if self.resends >= RESENDS {
            self.armed = None;
            return false;
        }
        let req = self.armed.clone().unwrap_or_default();
        self.echo.clear();
        self.matched.clear();
        self.same = 0;
        self.echo.extend(req.iter().copied());
        if self.wire.write(&req).is_ok() && self.wire.flush().is_ok() {
            self.resends += 1;
            self.t_armed = Instant::now();
            self.t_echo = self.t_armed;
            tracing::info!("K-line: no reply in 300 ms; request sent again ({})", self.resends);
        }
        true
    }

    /// Keep the negotiated rate: one `S` request, reply swallowed.
    fn keepalive(&mut self) {
        let now = Instant::now();
        self.t_tx = now;
        if self.ts_request(b"S", Duration::from_millis(400)).is_some() {
            self.t_rx = Instant::now();
        }
    }

    /// Sort bytes from the wire: our echo first, everything else is the ECU's.
    fn take(&mut self, data: &[u8]) {
        for &b in data {
            if let Some(want) = self.echo.pop_front() {
                self.matched.push(b);
                if want == b {
                    self.same += 1;
                }
                if self.echo.is_empty() {
                    self.echo_done();
                }
                continue;
            }
            self.armed = None; // a byte from the ECU: the request arrived
            self.rxq.push_back(b);
        }
    }

    /// As many bytes as we sent are back. Mostly equal = our own request, a
    /// difference means it was corrupted on the shared line, the ECU stays silent
    /// and these are not a reply: dropped, the re-send repeats it (plugin 2.3.0).
    /// Mostly different = an adapter without echo: this is the ECU's reply.
    fn echo_done(&mut self) {
        let got = std::mem::take(&mut self.matched);
        if self.same * 2 < got.len() {
            tracing::debug!("K-line: no echo on this cable ({} byte(s))", got.len());
            self.armed = None;
            self.rxq.extend(got);
        } else if self.same < got.len() {
            tracing::info!("K-line: request corrupted on the line, dropped");
        }
        self.same = 0;
    }

    /// Give up an echo that never came back.
    fn echo_timeout(&mut self) {
        if !self.echo.is_empty() && self.t_echo.elapsed() > ECHO_LOST {
            tracing::debug!("K-line: {} echo byte(s) never came back", self.echo.len());
            self.echo.clear();
            if self.same * 2 < self.matched.len() {
                // no echo: a reply shorter than the request
                let got = std::mem::take(&mut self.matched);
                self.armed = None;
                self.rxq.extend(got);
            }
            self.matched.clear();
            self.same = 0;
        }
    }
}

/// A [`CommunicationChannel`] over a K-line (see the module docs).
/// One KWP entry on a bare wire, for code that drives its own session on the
/// port (the firmware updater); no echo filter, no supervision. Hands the wire
/// back with the negotiated rate.
pub(crate) fn kwp_entry_once<W: KWire>(wire: W, rate: u32) -> (W, Result<u32, String>) {
    let now = Instant::now();
    let mut inner = Inner {
        wire,
        mode: KLineMode::Kwp,
        rate,
        echo: VecDeque::new(),
        rxq: VecDeque::new(),
        matched: Vec::new(),
        same: 0,
        frame: Vec::new(),
        armed: None,
        t_armed: now,
        resends: 0,
        t_echo: now,
        t_tx: now,
        t_rx: now,
        reply_limit: REPLY,
        timeout: Duration::from_millis(5),
        five_baud_bit: FIVE_BAUD_BIT,
    };
    let r = inner.kwp_attempt();
    (inner.wire, r)
}

pub struct KLineChannel<W: KWire + 'static = Box<dyn SerialPort>> {
    inner: Arc<Mutex<Inner<W>>>,
}

impl<W: KWire + 'static> KLineChannel<W> {
    /// Take an opened port, find the ECU (per `mode`) and start the session
    /// supervisor. `rate` is the link rate to use (direct) or ask for (KWP).
    pub fn open(wire: W, mode: KLineMode, rate: u32) -> Result<Self, ProtocolError> {
        Self::open_with_bit(wire, mode, rate, FIVE_BAUD_BIT)
    }

    fn open_with_bit(
        wire: W,
        mode: KLineMode,
        rate: u32,
        five_baud_bit: Duration,
    ) -> Result<Self, ProtocolError> {
        let now = Instant::now();
        let mut inner = Inner {
            wire,
            mode,
            rate,
            echo: VecDeque::new(),
            rxq: VecDeque::new(),
            matched: Vec::new(),
            same: 0,
            frame: Vec::new(),
            armed: None,
            t_armed: now,
            resends: 0,
            t_echo: now,
            t_tx: now,
            t_rx: now,
            reply_limit: REPLY,
            timeout: Duration::from_millis(100),
            five_baud_bit,
        };
        inner.enter().map_err(ProtocolError::ConnectionFailed)?;
        let inner = Arc::new(Mutex::new(inner));
        if mode != KLineMode::EchoOnly {
            let weak = Arc::downgrade(&inner);
            std::thread::Builder::new()
                .name("kline-supervisor".into())
                .spawn(move || supervise(weak))
                .map_err(|e| ProtocolError::ConnectionFailed(e.to_string()))?;
        }
        Ok(Self { inner })
    }

    /// The link rate in use (after a KWP entry: the negotiated one).
    pub fn rate(&self) -> u32 {
        lock(&self.inner).rate
    }

    /// Take the line, but give up after `limit` (an entry may hold it for seconds).
    fn acquire(&self, limit: Duration) -> io::Result<MutexGuard<'_, Inner<W>>> {
        let end = Instant::now() + limit;
        loop {
            match self.inner.try_lock() {
                Ok(g) => return Ok(g),
                Err(std::sync::TryLockError::Poisoned(p)) => return Ok(p.into_inner()),
                Err(std::sync::TryLockError::WouldBlock) => {
                    if Instant::now() >= end {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "K-line busy (entering the ECU)",
                        ));
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
            }
        }
    }

    fn timeout(&self) -> Duration {
        lock(&self.inner).timeout
    }
}

fn lock<W: KWire>(m: &Mutex<Inner<W>>) -> MutexGuard<'_, Inner<W>> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

/// Keepalive on a quiet line; re-entry when a request goes unanswered.
fn supervise<W: KWire>(weak: Weak<Mutex<Inner<W>>>) {
    loop {
        std::thread::sleep(Duration::from_millis(50));
        let Some(arc) = weak.upgrade() else { return };
        let Ok(mut g) = arc.try_lock() else { continue };
        let waiting = g.t_tx > g.t_rx;
        if waiting && g.resend_if_due() {
            continue;
        }
        if waiting && g.t_tx.elapsed() > g.reply_limit {
            if g.mode == KLineMode::Kwp {
                // OBD: the plugin went quiet after its two re-sends
                // (TunerStudio waited 8 s); leave the line alone first
                let quiet = QUIET.min(g.five_baud_bit * 13);
                std::thread::sleep(quiet);
            }
            // HaniCore fork (2026-11-04, 018DC car: a re-entry every ~5 s
            // over OBD). One lost reply on a shared line is not a lost ECU:
            // ask once at the session rate - the line has been left alone since
            // the last re-send - and enter again only if that is silent too.
            if g.ts_request(b"S", Duration::from_millis(400)).is_some() {
                tracing::warn!("K-line: a reply was lost; the ECU answers again at {} baud", g.rate);
                let _ = g.wire.clear_input();
                g.forget();
                continue;
            }
            tracing::warn!("K-line: the ECU stopped answering - entering again");
            // OBD: enter() tries the direct way first and leaves the line
            // QUIET before a 5-baud
            if let Err(e) = g.enter() {
                tracing::warn!("K-line: {}", e);
            }
            continue;
        }
        if !waiting
            && g.echo.is_empty()
            && g.rxq.is_empty()
            && g.t_rx.elapsed() > KEEPALIVE
            && g.t_tx.elapsed() > KEEPALIVE
        {
            g.keepalive();
        }
    }
}

impl<W: KWire + 'static> Read for KLineChannel<W> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let limit = self.timeout();
        let start = Instant::now();
        let mut g = self.acquire(limit)?;
        // our own bytes come back first; read past them to the ECU's
        while g.rxq.is_empty() {
            let mut tmp = [0u8; 256];
            let want = (g.echo.len() + buf.len()).clamp(1, tmp.len());
            match g.wire.read(&mut tmp[..want]) {
                Ok(n) => g.take(&tmp[..n]),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    g.echo_timeout();
                    if g.echo.is_empty() || start.elapsed() >= limit {
                        return Err(e);
                    }
                }
                Err(e) => return Err(e),
            }
        }
        let n = buf.len().min(g.rxq.len());
        for (dst, src) in buf.iter_mut().zip(g.rxq.drain(..n)) {
            *dst = src;
        }
        g.t_rx = Instant::now();
        Ok(n)
    }
}

impl<W: KWire + 'static> Write for KLineChannel<W> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let limit = self.timeout();
        let mut g = self.acquire(limit)?;
        // the echo is expected BEFORE the write: it can arrive before write() returns
        g.echo.extend(buf.iter().copied());
        let r = g.wire.write(buf);
        let n = *r.as_ref().unwrap_or(&0);
        let keep = g.echo.len() - (buf.len() - n);
        g.echo.truncate(keep);
        if n > 0 {
            g.t_echo = Instant::now();
            g.wrote(&buf[..n]);
            // an envelope starts with its 16-bit length, then the command
            if buf.len() >= 3 && buf[0] == 0 {
                g.reply_limit = if matches!(buf[2], b'B' | b'!') {
                    REPLY_SLOW
                } else {
                    REPLY
                };
            }
            g.t_tx = Instant::now();
        }
        r
    }

    fn flush(&mut self) -> io::Result<()> {
        let limit = self.timeout();
        self.acquire(limit)?.wire.flush()
    }
}

impl<W: KWire + 'static> CommunicationChannel for KLineChannel<W> {
    fn link_rate(&self) -> Option<u32> {
        Some(self.rate())
    }

    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        let mut g = self.acquire(Duration::from_secs(30))?;
        g.timeout = timeout;
        g.wire.set_timeout(timeout)
    }

    fn clear_input_buffer(&mut self) -> io::Result<()> {
        let limit = self.timeout();
        let mut g = self.acquire(limit)?;
        if !g.echo.is_empty() {
            // let the echo of what is still on the wire arrive, then drop all
            let bits = g.echo.len() as u64 * 10 * 1_000_000 / g.rate.max(1) as u64;
            std::thread::sleep(Duration::from_micros(bits) + Duration::from_millis(5));
            g.echo.clear();
        }
        g.matched.clear();
        g.same = 0;
        g.rxq.clear();
        g.wire.clear_input()
    }

    fn clear_output_buffer(&mut self) -> io::Result<()> {
        let limit = self.timeout();
        self.acquire(limit)?.wire.clear_output()
    }

    fn try_clone(&self) -> io::Result<Box<dyn CommunicationChannel>> {
        Ok(Box::new(KLineChannel {
            inner: Arc::clone(&self.inner),
        }))
    }

    fn bytes_to_read(&mut self) -> io::Result<u32> {
        let limit = self.timeout();
        let mut g = self.acquire(limit)?;
        // sort what has arrived, as far as the echo reaches
        let avail = g.wire.bytes_to_read()? as usize;
        if !g.echo.is_empty() && avail > 0 {
            let k = avail.min(g.echo.len());
            let got = g.read_raw(k, Duration::from_millis(50));
            g.take(&got);
        }
        g.echo_timeout();
        if !g.echo.is_empty() {
            return Ok(g.rxq.len() as u32);
        }
        Ok(g.rxq.len() as u32 + g.wire.bytes_to_read()?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A KKL cable on a HaniCore-like ECU: every written byte comes back as
    /// echo, then the ECU's answer. Rates and the 5-baud entry are modelled
    /// just far enough to drive the channel's state machine.
    struct FakeEcu {
        rx: VecDeque<u8>,
        baud: u32,
        ecu_baud: u32,
        breaks: Vec<bool>,
        kwp_stage: u8,
        refuse: Vec<u32>,
        echo: bool,
        pending: Vec<u8>,
        s_requests: Arc<Mutex<usize>>,
        /// non-'S' replies still to swallow (a reply lost on the line)
        drop: Arc<Mutex<usize>>,
        /// set_break calls: a 5-baud re-entry shows here
        break_calls: Arc<Mutex<usize>>,
        /// the car's line: closed, (last host activity, silence that opens it),
        /// and the requests sent while it was closed
        cut: Arc<Mutex<Option<(Instant, Duration)>>>,
        closed_writes: Arc<Mutex<usize>>,
        /// answers no envelope before a KWP entry (a line that only a 5-baud opens)
        needs_entry: bool,
        /// answers no 5-baud address (only the direct way in works)
        no_entry: bool,
    }

    impl FakeEcu {
        fn new(ecu_baud: u32) -> Self {
            Self {
                rx: VecDeque::new(),
                baud: 57600,
                ecu_baud,
                breaks: Vec::new(),
                kwp_stage: 0,
                refuse: Vec::new(),
                echo: true,
                pending: Vec::new(),
                s_requests: Arc::new(Mutex::new(0)),
                drop: Arc::new(Mutex::new(0)),
                break_calls: Arc::new(Mutex::new(0)),
                cut: Arc::new(Mutex::new(None)),
                closed_writes: Arc::new(Mutex::new(0)),
                needs_entry: false,
                no_entry: false,
            }
        }

        fn kwp_only(ecu_baud: u32) -> Self {
            let mut f = Self::new(ecu_baud);
            f.needs_entry = true;
            f
        }

        fn answer_envelope(&mut self) {
            if self.pending.len() < 2 {
                return;
            }
            let n = u16::from_be_bytes([self.pending[0], self.pending[1]]) as usize;
            if self.pending.len() < n + 6 {
                return;
            }
            let frame: Vec<u8> = self.pending.drain(..n + 6).collect();
            if self.baud != self.ecu_baud || (self.needs_entry && self.kwp_stage != 3) {
                return; // garbage at the wrong rate: the ECU says nothing
            }
            if frame[2] != b'S' {
                let mut d = self.drop.lock().unwrap();
                if *d > 0 {
                    *d -= 1;
                    return; // the reply was lost on the line
                }
            }
            let reply: &[u8] = match frame[2] {
                b'S' => {
                    *self.s_requests.lock().unwrap() += 1;
                    b"\x00rusEFI HaniCore ME711 TEST"
                }
                b'V' => b"\x00HaniCore TEST",
                _ => b"\x00\x11\x22\x33",
            };
            self.rx.extend(envelope(reply));
        }

        fn answer_kwp(&mut self, b: u8) {
            match self.kwp_stage {
                1 if b == 0x70 => {
                    self.rx.push_back(0xFE);
                    self.kwp_stage = 2;
                }
                2 => {
                    self.pending.push(b);
                    let len = (self.pending[0] & 0x3F) as usize + 4;
                    if self.pending.len() < len {
                        return;
                    }
                    let f = std::mem::take(&mut self.pending);
                    let data = &f[3..len - 1];
                    let reply: Vec<u8> = match data[0] {
                        0x81 => vec![0xC1, 0xEF, 0x8F],
                        0xB0 => [&[0xF0][..], b"HaniCore TEST"].concat(),
                        0xB1 => {
                            let rate = match data[1] {
                                0x01 => 10400,
                                0x02 => 57600,
                                0x03 => 124800,
                                _ => 115200,
                            };
                            if self.refuse.contains(&rate) {
                                vec![0x7F, 0xB1, 0x31]
                            } else {
                                self.ecu_baud = rate;
                                self.kwp_stage = 3;
                                vec![0xF1, data[1]]
                            }
                        }
                        _ => vec![0x7F, data[0], 0x11],
                    };
                    let mut out = vec![0x80 | reply.len() as u8, 0xF1, 0x01];
                    out.extend_from_slice(&reply);
                    out.push(out.iter().fold(0u8, |a, b| a.wrapping_add(*b)));
                    self.rx.extend(out);
                }
                _ => {}
            }
        }
    }

    impl FakeEcu {
        /// Closed until the host has been silent long enough; talking restarts it.
        fn line_closed(&mut self) -> bool {
            let mut c = self.cut.lock().unwrap();
            match *c {
                Some((t, q)) if t.elapsed() < q => {
                    *c = Some((Instant::now(), q));
                    true
                }
                Some(_) => {
                    *c = None;
                    false
                }
                None => false,
            }
        }
    }

    impl KWire for FakeEcu {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let n = buf.len().min(self.rx.len());
            if n == 0 {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "quiet"));
            }
            for b in buf.iter_mut().take(n) {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            if self.echo {
                self.rx.extend(buf.iter().copied());
            }
            if self.line_closed() {
                *self.closed_writes.lock().unwrap() += 1;
                return Ok(buf.len());
            }
            for &b in buf {
                if self.kwp_stage == 1 || self.kwp_stage == 2 {
                    self.answer_kwp(b);
                } else {
                    self.pending.push(b);
                    self.answer_envelope();
                }
            }
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn set_timeout(&mut self, _t: Duration) -> io::Result<()> {
            Ok(())
        }
        fn set_baud(&mut self, baud: u32) -> io::Result<()> {
            self.baud = baud;
            Ok(())
        }
        fn set_break(&mut self, on: bool) -> io::Result<()> {
            if self.line_closed() {
                return Ok(());
            }
            *self.break_calls.lock().unwrap() += 1;
            self.breaks.push(on);
            // the address byte 0x01 is complete: start bit, 8 data, stop
            let want = [
                true, false, true, true, true, true, true, true, true, false, false,
            ];
            if self.breaks.ends_with(&want) && !self.no_entry {
                self.breaks.clear();
                self.kwp_stage = 1;
                self.ecu_baud = 10400;
            }
            Ok(())
        }
        fn clear_input(&mut self) -> io::Result<()> {
            if self.kwp_stage == 1 && self.rx.is_empty() {
                // the ECU's sync + key bytes follow the address byte
                self.rx.extend([0x55, 0xEF, 0x8F]);
            } else {
                self.rx.clear();
            }
            Ok(())
        }
        fn clear_output(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn bytes_to_read(&mut self) -> io::Result<u32> {
            Ok(self.rx.len() as u32)
        }
    }

    fn roundtrip<W: KWire + 'static>(ch: &mut KLineChannel<W>, payload: &[u8]) -> Vec<u8> {
        ch.write_all(&envelope(payload)).unwrap();
        let mut got = Vec::new();
        let mut buf = [0u8; 64];
        let end = Instant::now() + Duration::from_secs(1);
        while Instant::now() < end {
            match ch.read(&mut buf) {
                Ok(n) => got.extend_from_slice(&buf[..n]),
                Err(_) => break,
            }
        }
        got
    }

    #[test]
    fn echo_filter_hides_the_cable_echo() {
        let mut ch = KLineChannel::open(FakeEcu::new(57600), KLineMode::EchoOnly, 57600).unwrap();
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn without_the_filter_the_echo_reaches_the_protocol() {
        // the control: the fake really echoes
        let mut w = FakeEcu::new(57600);
        w.write(&envelope(b"V")).unwrap();
        let mut buf = [0u8; 7];
        w.read(&mut buf).unwrap();
        assert_eq!(&buf[..], &envelope(b"V")[..]);
    }

    #[test]
    fn a_lost_echo_is_forgiven() {
        let mut w = FakeEcu::new(57600);
        w.echo = false;
        let mut ch = KLineChannel::open(w, KLineMode::EchoOnly, 57600).unwrap();
        ch.write_all(&envelope(b"V")).unwrap();
        std::thread::sleep(ECHO_LOST + Duration::from_millis(50));
        let mut buf = [0u8; 64];
        let n = ch.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], &envelope(b"\x00HaniCore TEST")[..]);
    }

    #[test]
    fn bytes_to_read_excludes_the_echo() {
        let mut ch = KLineChannel::open(FakeEcu::new(57600), KLineMode::EchoOnly, 57600).unwrap();
        ch.write_all(&envelope(b"V")).unwrap();
        assert_eq!(
            ch.bytes_to_read().unwrap() as usize,
            envelope(b"\x00HaniCore TEST").len()
        );
    }

    #[test]
    fn direct_finds_the_ecu_at_the_chosen_rate() {
        let mut ch = KLineChannel::open(FakeEcu::new(124800), KLineMode::Direct, 124800).unwrap();
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn direct_refuses_a_silent_line_quickly_enough_to_report() {
        // ECU at another rate: no answer, an error that says what to check
        let r = Inner {
            wire: FakeEcu::new(115200),
            mode: KLineMode::Direct,
            rate: 57600,
            echo: VecDeque::new(),
            rxq: VecDeque::new(),
            matched: Vec::new(),
            same: 0,
            frame: Vec::new(),
            armed: None,
            t_armed: Instant::now(),
            resends: 0,
            t_echo: Instant::now(),
            t_tx: Instant::now(),
            t_rx: Instant::now(),
            reply_limit: REPLY,
            timeout: Duration::from_millis(100),
            five_baud_bit: Duration::from_millis(1),
        }
        .find_link(Duration::from_millis(300));
        assert!(!r);
    }

    #[test]
    fn kwp_entry_negotiates_and_answers() {
        let mut ch = KLineChannel::open_with_bit(
            FakeEcu::kwp_only(57600),
            KLineMode::Kwp,
            124800,
            Duration::from_millis(1),
        )
        .unwrap();
        assert_eq!(ch.rate(), 124800);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn kwp_entry_can_stay_at_10400() {
        let mut ch = KLineChannel::open_with_bit(
            FakeEcu::kwp_only(57600),
            KLineMode::Kwp,
            10400,
            Duration::from_millis(1),
        )
        .unwrap();
        assert_eq!(ch.rate(), 10400);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn kwp_falls_back_when_the_rate_is_refused() {
        let mut w = FakeEcu::kwp_only(57600);
        w.refuse = vec![115200];
        let ch = KLineChannel::open_with_bit(w, KLineMode::Kwp, 115200, Duration::from_millis(1))
            .unwrap();
        assert_eq!(ch.rate(), 124800);
    }

    #[test]
    fn keepalive_on_a_quiet_line() {
        let w = FakeEcu::new(57600);
        let count = Arc::clone(&w.s_requests);
        let _ch = KLineChannel::open(w, KLineMode::Direct, 57600).unwrap();
        let after_entry = *count.lock().unwrap();
        std::thread::sleep(KEEPALIVE * 2 + Duration::from_millis(300));
        assert!(
            *count.lock().unwrap() > after_entry,
            "no keepalive was sent"
        );
    }

    #[test]
    fn a_lost_reply_costs_no_reentry() {
        // HaniCore fork, 2026-11-04 (018DC car: a re-entry every ~5 s over
        // OBD): one reply lost on the line is answered by a probe at the
        // session rate, not by a 5-baud entry
        let w = FakeEcu::new(57600);
        let drop = Arc::clone(&w.drop);
        let breaks = Arc::clone(&w.break_calls);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        let after_entry = *breaks.lock().unwrap();
        *drop.lock().unwrap() = 1;
        ch.write_all(&envelope(b"V")).unwrap();
        std::thread::sleep(REPLY + Duration::from_millis(1500));
        assert_eq!(*breaks.lock().unwrap(), after_entry, "a 5-baud re-entry ran");
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b" HaniCore TEST"));
    }

    #[test]
    fn a_lost_request_is_sent_again_within_a_second() {
        // the plugin's ReplyWatch: no reply byte in 300 ms -> the same request
        // again; the reply arrives with no probe and no re-entry
        let w = FakeEcu::new(57600);
        let drop = Arc::clone(&w.drop);
        let breaks = Arc::clone(&w.break_calls);
        let s = Arc::clone(&w.s_requests);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        let after_entry = *breaks.lock().unwrap();
        let s0 = *s.lock().unwrap();
        *drop.lock().unwrap() = 1;
        ch.write_all(&envelope(b"V")).unwrap();
        std::thread::sleep(Duration::from_millis(600));
        assert_eq!(*drop.lock().unwrap(), 0, "the request was not sent again");
        let want = envelope(b"\x00HaniCore TEST");
        let mut got = vec![0u8; want.len()];
        ch.set_timeout(Duration::from_millis(500)).unwrap();
        ch.read_exact(&mut got).unwrap();
        assert_eq!(got, want);
        assert_eq!(*breaks.lock().unwrap(), after_entry, "a 5-baud re-entry ran");
        assert_eq!(*s.lock().unwrap(), s0, "a probe ran");
    }

    #[test]
    fn a_cut_line_is_left_alone_and_comes_back() {
        // 2026-11-05 car: the line closes in a session, stays closed while the
        // host talks, opens after silence. 2 s of silence needed: the probe at
        // 2.5 s (1.9 s after the last re-send) is still too early, so the entry
        // must wait - exactly one entry, no hammering, the session back.
        let w = FakeEcu::kwp_only(57600);
        let breaks = Arc::clone(&w.break_calls);
        let cut = Arc::clone(&w.cut);
        let closed = Arc::clone(&w.closed_writes);
        let mut ch = KLineChannel::open(w, KLineMode::Kwp, 57600).unwrap();
        let before = *breaks.lock().unwrap();
        *cut.lock().unwrap() = Some((Instant::now(), Duration::from_secs(2)));
        ch.write_all(&envelope(b"V")).unwrap();
        // reply limit 2.5 s, then the plugin's quiet (2.5 s), then the probe
        std::thread::sleep(Duration::from_millis(8000));
        assert!(cut.lock().unwrap().is_none(), "the line never got its silence");
        assert!(*closed.lock().unwrap() <= 5, "hammered: {}", closed.lock().unwrap());
        assert_eq!(
            *breaks.lock().unwrap() - before,
            0,
            "after the silence the direct way back needs no 5-baud entry"
        );
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn a_short_cut_needs_no_entry_at_all() {
        // silence of 0.25 s is enough: the first re-send at 300 ms gets through
        let w = FakeEcu::new(57600);
        let breaks = Arc::clone(&w.break_calls);
        let cut = Arc::clone(&w.cut);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        let before = *breaks.lock().unwrap();
        *cut.lock().unwrap() = Some((Instant::now(), Duration::from_millis(250)));
        ch.write_all(&envelope(b"V")).unwrap();
        std::thread::sleep(Duration::from_millis(700));
        let want = envelope(b"\x00HaniCore TEST");
        let mut got = vec![0u8; want.len()];
        ch.read_exact(&mut got).unwrap();
        assert_eq!(got, want);
        assert_eq!(*breaks.lock().unwrap(), before, "a 5-baud re-entry ran");
    }

    #[test]
    fn the_echo_of_a_resend_is_not_a_reply() {
        // 2026-11-05 car: "OCH response: 5 bytes", "8 of 1363 bytes" - the echo of
        // the re-sent request went up as the ECU's reply
        let w = FakeEcu::new(57600);
        let drop = Arc::clone(&w.drop);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        *drop.lock().unwrap() = 2; // lost, and the first re-send lost too
        ch.write_all(&envelope(b"V")).unwrap();
        std::thread::sleep(Duration::from_millis(900));
        let want = envelope(b"\x00HaniCore TEST");
        let mut got = vec![0u8; want.len()];
        ch.read_exact(&mut got).unwrap();
        assert_eq!(got, want, "the caller read something else than the reply");
    }

    #[test]
    fn obd_enters_direct_first() {
        // an ECU that would answer both ways: DIRECT, and no 5-baud at all
        // (plugin 2.3.0 AUTO order; every car session that died had a 5-baud)
        let w = FakeEcu::new(57600);
        let breaks = Arc::clone(&w.break_calls);
        let s = Arc::clone(&w.s_requests);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        assert_eq!(*breaks.lock().unwrap(), 0, "a 5-baud was sent although DIRECT answers");
        assert!(*s.lock().unwrap() >= 1, "no direct probe");
        assert_eq!(ch.rate(), 57600);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b" HaniCore TEST"));
    }

    #[test]
    fn obd_direct_falls_to_the_boot_rate() {
        // a 24 MHz board has no 115200: DIRECT at the boot rate, no 5-baud
        let mut w = FakeEcu::new(57600);
        w.no_entry = true;
        let breaks = Arc::clone(&w.break_calls);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 115200, Duration::from_millis(1))
                .unwrap();
        assert_eq!(*breaks.lock().unwrap(), 0, "a 5-baud was sent");
        assert_eq!(ch.rate(), 57600);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b" HaniCore TEST"));
    }

    #[test]
    fn obd_10400_goes_by_5_baud() {
        // 10400 = stay at the entry rate (what VCDS does): the 5-baud entry
        let w = FakeEcu::new(57600);
        let breaks = Arc::clone(&w.break_calls);
        let ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 10400, Duration::from_millis(1))
                .unwrap();
        assert_eq!(*breaks.lock().unwrap(), 11, "not exactly one 5-baud entry");
        assert_eq!(ch.rate(), 10400);
    }

    #[test]
    fn obd_kwp_only_line() {
        // a line that only a 5-baud opens
        let w = FakeEcu::kwp_only(57600);
        let breaks = Arc::clone(&w.break_calls);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 124800, Duration::from_millis(1))
                .unwrap();
        assert_eq!(*breaks.lock().unwrap(), 11, "not exactly one 5-baud entry");
        assert_eq!(ch.rate(), 124800);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b" HaniCore TEST"));
    }

    #[test]
    fn a_burn_is_never_sent_again() {
        let w = FakeEcu::new(57600);
        let drop = Arc::clone(&w.drop);
        let mut ch =
            KLineChannel::open_with_bit(w, KLineMode::Kwp, 57600, Duration::from_millis(1))
                .unwrap();
        *drop.lock().unwrap() = 2;
        ch.write_all(&envelope(b"B")).unwrap();
        std::thread::sleep(Duration::from_millis(800));
        assert_eq!(*drop.lock().unwrap(), 1, "a Burn was repeated");
    }

    #[test]
    fn kwp_frame_matches_the_reference() {
        // me711_kwp_connect.kwp(b"\x81") == 81 01 F1 81 F4
        assert_eq!(kwp_frame(&[0x81]), vec![0x81, 0x01, 0xF1, 0x81, 0xF4]);
    }
}
