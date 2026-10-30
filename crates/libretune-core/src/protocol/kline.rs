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
fn envelope(payload: &[u8]) -> Vec<u8> {
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
    /// Bytes matched against the current echo so far.
    matched: Vec<u8>,
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
        self.five_baud(0x01).map_err(|e| e.to_string())?;
        let sync = self.read_raw(3, Duration::from_millis(400));
        if sync != [0x55, 0xEF, 0x8F] {
            return Err(format!("no 5-baud sync ({:02X?})", sync));
        }
        std::thread::sleep(Duration::from_millis(30)); // W4: 25..50 ms
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
        let mut rate = if rate_code(self.rate).is_some() && self.rate != 10400 {
            self.rate
        } else {
            124800
        };
        let mut d = self.kwp_request(
            &[0xB1, rate_code(rate).unwrap()],
            Duration::from_millis(300),
        );
        // a refused rate (7F B1 ..: the ECU cannot make it) - same session, the next one
        for alt in [57600u32, 124800] {
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

    /// (Re-)enter: find the ECU and leave the line clean.
    fn enter(&mut self) -> Result<(), String> {
        let r = match self.mode {
            KLineMode::EchoOnly => Ok(()),
            KLineMode::Direct => {
                if self.find_link(FIND_LINK) {
                    Ok(())
                } else {
                    Err(format!(
                        "no answer at {} baud - ECU powered? cable on the K-line pin?",
                        self.rate
                    ))
                }
            }
            KLineMode::Kwp => {
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
                            // the ECU falls back to KWP idle on silence
                            std::thread::sleep(Duration::from_millis(2500));
                        }
                    }
                }
                match ok {
                    Some(rate) => {
                        tracing::info!("K-line: vehicle session at {} baud", rate);
                        self.rate = rate;
                        Ok(())
                    }
                    None => Err(format!(
                        "no KWP entry ({}) - ignition on? cable in the OBD socket?",
                        last
                    )),
                }
            }
        };
        let _ = self.wire.clear_input();
        self.echo.clear();
        self.matched.clear();
        self.rxq.clear();
        let now = Instant::now();
        self.t_tx = now;
        self.t_rx = now;
        r
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
            if self.echo.front() == Some(&b) {
                self.echo.pop_front();
                self.matched.push(b);
                if self.echo.is_empty() {
                    self.matched.clear();
                }
                continue;
            }
            if !self.echo.is_empty() {
                // no echo after all: what looked like its start (every
                // envelope begins with 0x00) was the ECU's
                tracing::debug!("K-line: echo lost ({} byte(s))", self.echo.len());
                self.echo.clear();
                let back = std::mem::take(&mut self.matched);
                self.rxq.extend(back);
            }
            self.rxq.push_back(b);
        }
    }

    /// Give up an echo that never came back.
    fn echo_timeout(&mut self) {
        if !self.echo.is_empty() && self.t_tx.elapsed() > ECHO_LOST {
            tracing::debug!("K-line: {} echo byte(s) never came back", self.echo.len());
            self.echo.clear();
            self.matched.clear();
        }
    }
}

/// A [`CommunicationChannel`] over a K-line (see the module docs).
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
        if waiting && g.t_tx.elapsed() > g.reply_limit {
            tracing::warn!("K-line: the ECU stopped answering - entering again");
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
            }
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
            if self.baud != self.ecu_baud {
                return; // garbage at the wrong rate: the ECU says nothing
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
            self.breaks.push(on);
            // the address byte 0x01 is complete: start bit, 8 data, stop
            let want = [
                true, false, true, true, true, true, true, true, true, false, false,
            ];
            if self.breaks.len() == want.len() && self.breaks == want {
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
            FakeEcu::new(57600),
            KLineMode::Kwp,
            124800,
            Duration::from_millis(1),
        )
        .unwrap();
        assert_eq!(ch.rate(), 124800);
        assert_eq!(roundtrip(&mut ch, b"V"), envelope(b"\x00HaniCore TEST"));
    }

    #[test]
    fn kwp_falls_back_when_the_rate_is_refused() {
        let mut w = FakeEcu::new(57600);
        w.refuse = vec![115200];
        let ch = KLineChannel::open_with_bit(w, KLineMode::Kwp, 115200, Duration::from_millis(1))
            .unwrap();
        assert_eq!(ch.rate(), 57600);
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
    fn kwp_frame_matches_the_reference() {
        // me711_kwp_connect.kwp(b"\x81") == 81 01 F1 81 F4
        assert_eq!(kwp_frame(&[0x81]), vec![0x81, 0x01, 0xF1, 0x81, 0xF4]);
    }
}
