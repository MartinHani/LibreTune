// Added 2026-11-03 in the HaniCore K-line fork of LibreTune (see FORK.md).
//! The runtime channels the screen shows (status bar, readouts, gauges, live
//! graphs): their byte ranges are read on every poll, the rest of the block a
//! slice per poll (K-line is slow - the whole block is ~0.3 s).

use crate::state::AppState;
use libretune_core::ini::OutputChannel;
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// What the screen shows (set_runtime_focus) and what a recording log
/// writes (data_logging): the focus is their union.
static SCREEN: Mutex<Vec<String>> = Mutex::new(Vec::new());
static LOGGED: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// A recording log's channels join the focus (empty = none).
pub(crate) async fn set_logged_channels(state: &AppState, channels: Vec<String>) {
    *LOGGED.lock().unwrap() = channels;
    apply_focus(state).await;
}

/// Ranges closer than this many bytes are read as one: on K-line a request
/// costs about as much as ~100 bytes of reply, so a gap is cheaper than it.
const MERGE_GAP: usize = 64;

/// At most this many requests per poll: nearby ranges are merged until it fits.
const MAX_RANGES: usize = 5;

/// The block channels a channel reads: itself, or for a computed channel the
/// channels its expression names (three levels deep).
fn collect(
    name: &str,
    chans: &HashMap<String, OutputChannel>,
    depth: u8,
    seen: &mut HashSet<String>,
    out: &mut Vec<(usize, usize)>,
) {
    if depth > 3 || !seen.insert(name.to_string()) {
        return;
    }
    let Some(c) = chans.get(name) else { return };
    if c.size_bytes() > 0 {
        out.push((c.offset as usize, c.offset as usize + c.size_bytes()));
        return;
    }
    if let Some(expr) = &c.expression {
        let mut word = String::new();
        for ch in expr.chars().chain(std::iter::once(' ')) {
            if ch.is_ascii_alphanumeric() || ch == '_' {
                word.push(ch);
            } else if !word.is_empty() {
                if !word.chars().next().unwrap().is_ascii_digit() {
                    collect(&word, chans, depth + 1, seen, out);
                }
                word.clear();
            }
        }
    }
}

/// `channels` = every live channel on screen (empty = none).
#[tauri::command]
pub async fn set_runtime_focus(
    state: tauri::State<'_, AppState>,
    channels: Vec<String>,
) -> Result<usize, String> {
    *SCREEN.lock().unwrap() = channels;
    Ok(apply_focus(&state).await)
}

async fn apply_focus(state: &AppState) -> usize {
    let mut channels: Vec<String> = SCREEN.lock().unwrap().clone();
    channels.extend(LOGGED.lock().unwrap().iter().cloned());
    let mut spans: Vec<(usize, usize)> = Vec::new();
    {
        let def_guard = state.definition.lock().await;
        let def = match def_guard.as_ref() {
            Some(d) => d,
            None => return 0,
        };
        let mut seen = HashSet::new();
        for n in &channels {
            collect(n, &def.output_channels, 0, &mut seen, &mut spans);
        }
    }
    spans.sort();
    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (a, b) in spans {
        match merged.last_mut() {
            Some(last) if a <= last.1 + MERGE_GAP => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    // too many requests: close the smallest gap until it fits
    while merged.len() > MAX_RANGES {
        let mut best = 1;
        for i in 1..merged.len() {
            if merged[i].0 - merged[i - 1].1 < merged[best].0 - merged[best - 1].1 {
                best = i;
            }
        }
        merged[best - 1].1 = merged[best].1;
        merged.remove(best);
    }
    // a focus that is most of the block is no focus: whole reads are cheaper
    if merged.iter().map(|(a, b)| b - a).sum::<usize>() > 900 {
        merged.clear();
    }
    let ranges: Vec<(u16, u16)> = merged
        .iter()
        .map(|(a, b)| (*a as u16, (*b - *a) as u16))
        .collect();
    let n = ranges.len();
    let mut conn_guard = state.connection.lock().await;
    if let Some(conn) = conn_guard.as_mut() {
        conn.set_och_focus(ranges);
    }
    n
}
