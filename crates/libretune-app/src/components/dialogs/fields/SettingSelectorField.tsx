// Modified 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
import { useEffect, useMemo, useState } from 'react';
import { invoke } from '@tauri-apps/api/core';
import { emit } from '@tauri-apps/api/event';
import type { DialogComponent } from '../types';
import { useDialogValueSource } from '../DialogValueSource';

/// Preset dropdown (TunerStudio `settingSelector`): each option applies a set
/// of `constant = value` assignments. When the current values don't match any
/// option, the dropdown shows the synthetic "Custom" entry.
export function SettingSelectorField({
  comp,
  context,
  onUpdate,
  onOptimisticUpdate,
}: {
  comp: DialogComponent;
  context: Record<string, number>;
  onUpdate?: () => void;
  onOptimisticUpdate?: (name: string, value: number) => void;
}) {
  const valueSource = useDialogValueSource();
  const readOnly = !!valueSource?.readOnly;
  const [fetched, setFetched] = useState<Record<string, number>>({});
  const [visible, setVisible] = useState(true);

  const constantNames = useMemo(() => {
    const names = new Set<string>();
    comp.options?.forEach((o) => o.assignments.forEach((a) => names.add(a.name)));
    return [...names];
  }, [comp.options]);

  useEffect(() => {
    if (valueSource) return; // live values already provided by the tune editor
    let cancelled = false;
    const load = async () => {
      const entries: Record<string, number> = {};
      for (const n of constantNames) {
        try {
          entries[n] = await invoke<number>('get_constant_value', { name: n });
        } catch {
          // Constant not loaded — treated as non-matching
        }
      }
      if (!cancelled) setFetched(entries);
    };
    load();
    return () => {
      cancelled = true;
    };
  }, [constantNames, valueSource]);

  useEffect(() => {
    if (!comp.visibility_condition) {
      setVisible(true);
      return;
    }
    invoke<boolean>('evaluate_expression', { expression: comp.visibility_condition, context })
      .then(setVisible)
      .catch(() => setVisible(true));
  }, [comp.visibility_condition, context]);

  const values: Record<string, number> = valueSource ? valueSource.numbers : fetched;

  const matchedIndex = useMemo(() => {
    if (!comp.options || Object.keys(values).length === 0) return -1;
    for (let i = 0; i < comp.options.length; i++) {
      const allMatch = comp.options[i].assignments.every((a) => {
        const v = values[a.name];
        return v !== undefined && Math.abs(v - a.value) < 1e-6 * Math.max(1, Math.abs(a.value));
      });
      if (allMatch) return i;
    }
    return -1;
  }, [comp.options, values]);

  if (!visible) return null;

  const handleChange = async (e: React.ChangeEvent<HTMLSelectElement>) => {
    const idx = parseInt(e.target.value, 10);
    if (Number.isNaN(idx) || idx < 0 || readOnly) return;
    const opt = comp.options?.[idx];
    if (!opt) return;
    for (const a of opt.assignments) {
      try {
        await invoke('update_constant', { name: a.name, value: a.value });
        onOptimisticUpdate?.(a.name, a.value);
      } catch (err) {
        console.error(`[SettingSelector] update_constant '${a.name}' failed:`, err);
      }
    }
    setFetched((prev) => {
      const next = { ...prev };
      for (const a of opt.assignments) next[a.name] = a.value;
      return next;
    });
    onUpdate?.();
    // A preset usually fills a curve or table shown beside it (HaniCore's
    // sensor pickers fill the curve in the same dialog); those panels re-read
    // their data only on tune:loaded, so without this they kept showing the
    // old values although the ECU already held the new ones.
    emit('tune:loaded', 'setting-selector').catch(() => {});
  };

  return (
    <div className="settings-field dialog-field">
      <label className="field-label">
        <span className="field-label-text">{comp.label}</span>
      </label>
      <div className="field-input-wrap">
        <select value={matchedIndex >= 0 ? matchedIndex : -1} disabled={readOnly} onChange={handleChange}>
          <option value={-1}>Custom</option>
          {comp.options?.map((o, i) => (
            <option key={i} value={i}>
              {o.label}
            </option>
          ))}
        </select>
      </div>
    </div>
  );
}
