# LibreTune — HaniCore K-line build (unofficial fork)

This is a **modified version of [LibreTune](https://github.com/RallyPat/LibreTune)**,
maintained for [HaniCore](https://github.com/MartinHani/HaniCore) (standalone
firmware for Bosch ME7 ECUs). It is **not an official LibreTune release** and
is not endorsed by the LibreTune project. Report problems with it to the
HaniCore project, not to LibreTune.

It adds **native K-line support** so a HaniCore ECU can be tuned with an
ordinary KKL / VAG-COM cable, and nothing else of substance. The K-line code
is specific to HaniCore's entry protocol and is kept in this fork rather than
offered to the main LibreTune build.

## License

LibreTune is licensed under the **GNU General Public License, version 2**
(`LICENSE`); this fork keeps that license unchanged, and every change below is
released under it. Third-party components keep their own licenses
(`THIRD_PARTY_NOTICES.md`). The complete corresponding source of every binary
published from this fork is the tagged commit of this repository named in
that release.

## Base

Upstream `RallyPat/LibreTune` `master` at commit `cc5cf23` (2026-10-30).

## Changes (GPL-2.0 section 2a)

| date | file | change |
|---|---|---|
| 2026-10-30 | `crates/libretune-core/src/protocol/kline.rs` | **new**: `KLineChannel` — echo filter matched by content, direct rate hunt, 5-baud + ISO 14230 / KWP entry with GOFAST, keepalive and automatic re-entry; unit tests against a simulated cable + ECU |
| 2026-10-30 | `crates/libretune-core/src/protocol/connection.rs` | `ConnectionType::KLine` / `KLineKwp`; serial port wrapped in `KLineChannel`; `filterEchoBytes = true` in an INI applies the echo filter to a plain serial connection; no post-open settle delay for K-line |
| 2026-10-30 | `crates/libretune-core/src/protocol/mod.rs` | `kline` module exported |
| 2026-10-30 | `crates/libretune-app/src-tauri/src/commands/connect_to_ecu.rs` | connection types `"KLine"` / `"KLineKwp"` |
| 2026-10-30 | `crates/libretune-app/src/components/tuner-ui/dialogs/ConnectionDialog.tsx` | **K-line** mode with "Cable connected to: ECU pin / car OBD socket"; 124800 baud for it |
| 2026-10-30 | `crates/libretune-app/src/components/dialogs/ConnectEcuWizard.tsx`, `src/utils/connectEcuWizard.ts` | K-line transport in the Connect-ECU / New Project wizard, with the same choice and the K-line rates |
| 2026-10-30 | `crates/libretune-app/src/App.tsx`, `src/components/DialogOverlays.tsx`, `src/hooks/useAutoConnect.ts` | connection-type union widened |
| 2026-10-30 | `crates/libretune-core/src/pin_conflict.rs` | `[PcVariables]` are never treated as pin selectors (false "pin conflict" before every Burn on HaniCore's trouble-code list pickers) |
| 2026-10-30 | `crates/libretune-app/src/components/tuner-ui/__tests__/ConnectionDialog.test.tsx`, `src/utils/__tests__/connectEcuWizard.test.ts` | tests for the above |
| 2026-10-31 | `crates/libretune-app/src-tauri/tauri.conf.json` | version `0.1.0-hanicore.1`, identifier `com.hanicore.libretune` (its own settings, so it does not collide with an official LibreTune install; projects in `Documents\LibreTuneProjects` are shared), window title marks the build unofficial |
| 2026-10-31 | `crates/libretune-app/src/hooks/useEcuEventListeners.ts` | window title "LibreTune (HaniCore build)" |
| 2026-10-31 | `README.md`, `CHANGELOG.md`, `FORK.md` | this notice and the change log |
| 2026-11-01 | `crates/libretune-core/src/ini/constants.rs`, `ini/output_channels.rs`, `action_scripting.rs`, `simulator/och_codec.rs`, `simulator/ve_model.rs`, `crates/libretune-app/src-tauri/src/commands/tune_compare.rs`, `src/components/dashboards/designer/useDesignerDrop.ts` | **scale/translate**: the INI rule is `userValue = (msValue + translate) * scale` (TunerStudio; stated in Speeduino's INI header) - LibreTune computed `raw * scale + translate`, which agrees only at scale 1. Every field with both (HaniCore: temperatures `10*C + 10000`, the ignition map, the throttle bias curve - 91 page fields and the temperature gauges) read and wrote wrong; a sensor picker wrote 65535. Tune merge also wrote only the low byte of a field |
| 2026-11-01 | `crates/libretune-core/src/ini/mod.rs`, `crates/libretune-app/src-tauri/src/commands/constant_update.rs`, `curve_ops.rs` | `overlapping_constant_names()`: a write through one name drops the stale `tune.constants` entry of every other name for the same bytes (an array and the scalars a `settingOption` assigns); a curve write the ECU refused is an error, not a log line |
| 2026-11-01 | `crates/libretune-core/src/tune/file.rs` | a tune file saved before the scale/translate fix (manifest without `valueRule="tunerstudio"`) has those values re-expressed on load; new saves carry the marker |
| 2026-11-01 | `crates/libretune-app/src/components/dialogs/fields/SettingSelectorField.tsx` | after a preset choice emit `tune:loaded`, so a curve/table shown beside it re-reads (the coolant curve kept showing the old values though the ECU held the new ones) |
| 2026-11-01 | `crates/libretune-app/src-tauri/tauri.conf.json` | version `0.1.0-hanicore.2` |
| 2026-11-02 | `crates/libretune-core/src/protocol/connection.rs` | K-line table writes in 64-byte frames, each read back: a 256-byte frame was longer than the ECU's receive buffer (72) and was dropped without a reply - VE / ignition edits did not reach the ECU |
| 2026-11-02 | `crates/libretune-app/src-tauri/src/commands/sync_ecu_data.rs` | a new project has no Difference Report against an empty cache; "Ignore" no longer leaves a tune without pages |
| 2026-11-02 | `crates/libretune-core/src/protocol/kline.rs`, `commands/connect_to_ecu.rs` | K-line default 57600; a direct K-line connect falls back to 57600; the wizard's detect step does not pop the INI-mismatch dialog |
| 2026-11-02 | `crates/libretune-app/src-tauri/src/commands/signature_helpers.rs` | signatures that share a layout id match only with the same board name (a Porsche ECU was paired with the ME7.5 file) |
| 2026-11-02 | `crates/libretune-app/src/App.tsx`, `commands/project_lifecycle.rs` | switching project updates the header, window title and last project; after a connect the project keeps the baud rate that was USED (it kept the previous one) |
| 2026-11-02 | `crates/libretune-app/src-tauri/src/commands/app_settings.rs`, `settings.rs` | the connect dialog remembers the last connection type |
| 2026-11-02 | `crates/libretune-app/src/utils/evaluateIndicatorExpression.ts` (+ test), `dialogs/fields/IndicatorPanelRenderer.tsx` | indicator expressions by an own parser: `new Function` is blocked by the release build's content-security policy, so every indicator comparison was false |
| 2026-11-02 | `crates/libretune-core/src/ini/expression.rs`, `commands/ini_dialogs.rs` | `{ TEXT bitStringValue(list, x) }` labels interpolate; label evaluation folds computed channels but keeps live values; panels subscribe the label variables |
| 2026-11-02 | `crates/libretune-core/src/ini/parser.rs` | `readout = gaugeName` resolves the gauge (every readout panel was empty) |
| 2026-11-02 | `crates/libretune-core/src/project/repository.rs` | importing an INI whose signature is already known replaces it |
| 2026-11-02 | `crates/libretune-app/src/components/dialogs/fields/DialogField.tsx` | number fields: no floating-point noise, no ECU write when a field loses focus untouched |
| 2026-11-02 | `crates/libretune-app/src-tauri/src/commands/page_refresh.rs` (new), `src/services/realtimeListener.ts`, `lib.rs`, `commands/mod.rs` | `[EventTriggers] triggeredPageRefresh`: the page is re-read when the ECU raises the refresh channel (after AutoCal, a preset, a Grab) - the PID fields now update by themselves after AutoCal |
| 2026-11-02 | `crates/libretune-app/src/components/dialogs/ConnectEcuWizard.tsx` | the wizard offers the last port that worked (it offered the first one, COM1) |
| 2026-11-02 | `crates/libretune-app/src/services/openTarget.ts`, `components/curves/CurveEditor.tsx` | window titles show the menu label only, not the INI's internal name in brackets |
| 2026-11-02 | `crates/libretune-app/src/components/tuner-ui/dialogs/BurnDialog.tsx`, `components/DialogOverlays.tsx` | the Burn window closes after a successful Burn (it stayed open, Burn still live); the project's tune file is saved at the Burn, so the next connect shows no Difference Report against the old file |
| 2026-11-02 | `crates/libretune-app/src-tauri/tauri.conf.json` | version `0.1.0-hanicore.3` |

Each modified source file also carries a one-line notice at its top.

## Build

As upstream (see `CONTRIBUTING.md`): Rust stable, Node 20+, the Tauri
prerequisites; then in `crates/libretune-app`: `npm install`,
`npm run tauri -- build --no-bundle` → `target/release/libretune-app.exe`.

## Using it with HaniCore

Connection → **K-line** → cable connected to **ECU K-line pin** (bench) or
**car OBD socket** → COM port → 57600 or 124800 → Connect; import the board's
`.ini` from the HaniCore release when asked for the ECU definition.
