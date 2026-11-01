// Modified 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import { useRealtimeStore } from "../stores/realtimeStore";
import { evalExpression } from "../utils/evaluateIndicatorExpression";

// Registered once and never unregistered — prevents race conditions when React
// effects re-run during connect/disconnect cycles.
let _realtimeListenerPromise: Promise<void> | null = null;

// [EventTriggers] triggeredPageRefresh: the ECU raises a channel when it has
// changed a page itself (preset APPLY, a Grab, an ETB calibration). On the
// rising edge the page is read again; without it the app kept showing the
// old values and would later have written them back.
let triggers: Array<[number, string]> = [];
let triggersAt = 0;
const wasTrue = new Map<string, boolean>();
let refreshing = false;

async function checkPageRefresh(channels: Record<string, number>) {
  const now = Date.now();
  if (now - triggersAt > 5000) {
    triggersAt = now;
    triggers = await invoke<Array<[number, string]>>("get_page_refresh_triggers").catch(() => []);
  }
  for (const [page, expr] of triggers) {
    let on = false;
    try {
      on = evalExpression(expr, channels) !== 0;
    } catch {
      on = false;
    }
    const key = `${page}:${expr}`;
    if (on && !wasTrue.get(key) && !refreshing) {
      refreshing = true;
      invoke("refresh_page_from_ecu", { page })
        .catch((e) => console.warn("[page refresh]", e))
        .finally(() => {
          refreshing = false;
        });
    }
    wasTrue.set(key, on);
  }
}

export function ensureRealtimeListener(): Promise<void> {
  if (_realtimeListenerPromise) return _realtimeListenerPromise;
  _realtimeListenerPromise = (async () => {
    await listen("realtime:update", (event) => {
      const channels = event.payload as Record<string, number>;
      useRealtimeStore.getState().updateChannels(channels);
      void checkPageRefresh(channels);
    });
    await listen("realtime:error", (event) => {
      console.error("Realtime error:", event.payload);
    });
  })();
  return _realtimeListenerPromise;
}
