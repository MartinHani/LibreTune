// Added 2026-11-03 in the HaniCore K-line fork of LibreTune (see FORK.md).
/**
 * LiveGraph - draws an .ini `liveGraph = id, "title", where` + `graphLine = ch`
 * block as a scrolling time trace (HaniCore fork; was a placeholder).
 *
 * Each line is scaled to its own visible min/max (channels of different units
 * share one strip, like a TunerStudio dialog graph); the legend shows the
 * gauge title, the live value and the visible range. Samples come from the
 * realtime store's history buffer (one entry per realtime update), read each
 * animation frame - nothing here causes React re-renders.
 */
import { useEffect, useRef, useState } from 'react';
import { invoke } from '@tauri-apps/api/core';
import { getChannelHistoryBuffer, useOnScreenChannels, useRealtimeStore } from '../../../stores/realtimeStore';
import type { SimpleGaugeInfo } from '../../curves/CurveEditor';

const COLORS = ['#38bdf8', '#f97316', '#a3e635', '#f472b6', '#facc15', '#c084fc'];

interface LineMeta {
  channel: string;
  label: string;
  units: string;
  digits: number;
}

let gaugeCache: Promise<SimpleGaugeInfo[]> | null = null;
function allGauges(): Promise<SimpleGaugeInfo[]> {
  if (!gaugeCache) {
    gaugeCache = invoke<SimpleGaugeInfo[]>('get_gauge_configs').catch(() => {
      gaugeCache = null;
      return [];
    });
  }
  return gaugeCache;
}

function latest(channel: string): number | undefined {
  const ch = useRealtimeStore.getState().channels;
  if (ch[channel] !== undefined) return ch[channel];
  const lower = channel.toLowerCase();
  for (const k of Object.keys(ch)) if (k.toLowerCase() === lower) return ch[k];
  return undefined;
}

function fmt(v: number | undefined, digits: number): string {
  if (v === undefined || !Number.isFinite(v)) return '--';
  return v.toFixed(Math.max(0, Math.min(3, digits)));
}

export function LiveGraph({ title, channels }: { title?: string; channels: string[] }) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [meta, setMeta] = useState<LineMeta[]>(
    channels.map((c) => ({ channel: c, label: c, units: '', digits: 1 })),
  );
  const legendRefs = useRef<(HTMLSpanElement | null)[]>([]);

  useEffect(() => {
    let cancelled = false;
    allGauges().then((gs) => {
      if (cancelled) return;
      setMeta(
        channels.map((c) => {
          const g = gs.find((x) => x.channel === c) || gs.find((x) => x.name === `${c}Gauge`);
          return g
            ? { channel: c, label: g.title || c, units: g.units || '', digits: g.digits ?? 1 }
            : { channel: c, label: c, units: '', digits: 1 };
        }),
      );
    });
    return () => {
      cancelled = true;
    };
  }, [channels.join('|')]);

  useOnScreenChannels(channels);

  useEffect(() => {
    let raf = 0;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const cv = canvasRef.current;
      const wrap = wrapRef.current;
      if (!cv || !wrap) return;
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(100, wrap.clientWidth);
      const h = 160;
      if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
        cv.style.width = `${w}px`;
        cv.style.height = `${h}px`;
      }
      const ctx = cv.getContext('2d');
      if (!ctx) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const style = getComputedStyle(wrap);
      const bg = style.getPropertyValue('--bg-input').trim() || '#111827';
      const grid = style.getPropertyValue('--border').trim() || '#374151';
      ctx.fillStyle = bg;
      ctx.fillRect(0, 0, w, h);
      ctx.strokeStyle = grid;
      ctx.lineWidth = 1;
      for (let i = 1; i < 4; i++) {
        const y = Math.round((h * i) / 4) + 0.5;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }
      const pad = 6;
      meta.forEach((m, idx) => {
        const hist = getChannelHistoryBuffer(m.channel);
        const span = legendRefs.current[idx];
        const now = latest(m.channel);
        if (!hist || hist.length < 2) {
          if (span) span.textContent = `${fmt(now, m.digits)} ${m.units}`.trim();
          return;
        }
        let lo = Infinity;
        let hi = -Infinity;
        for (const v of hist) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
        const n = hist.length;
        const flat = hi - lo < 1e-9;
        const range = flat ? 1 : hi - lo;
        // A constant line gets its own lane (top to bottom by legend order) so
        // several flat lines do not hide each other on one middle line.
        const lane = (idx + 1) / (meta.length + 1);
        ctx.strokeStyle = COLORS[idx % COLORS.length];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
          const x = w - ((n - 1 - i) / 299) * w;
          const f = flat ? 1 - lane : (hist[i] - lo) / range;
          const y = pad + (1 - f) * (h - 2 * pad);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
        if (span) {
          span.textContent =
            `${fmt(now ?? hist[n - 1], m.digits)} ${m.units}`.trim() +
            `  [${fmt(lo, m.digits)} .. ${fmt(hi, m.digits)}]`;
        }
      });
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [meta]);

  return (
    <div className="embedded-live-graph" ref={wrapRef}>
      {title && <div className="embedded-live-graph-title">{title}</div>}
      <canvas ref={canvasRef} />
      <div className="embedded-live-graph-legend">
        {meta.map((m, i) => (
          <span key={m.channel} className="embedded-live-graph-item">
            <i style={{ background: COLORS[i % COLORS.length] }} />
            {m.label}:{' '}
            <span
              className="embedded-live-graph-value"
              ref={(el) => {
                legendRefs.current[i] = el;
              }}
            />
          </span>
        ))}
      </div>
    </div>
  );
}
