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

Each modified source file also carries a one-line notice at its top.

## Build

As upstream (see `CONTRIBUTING.md`): Rust stable, Node 20+, the Tauri
prerequisites; then in `crates/libretune-app`: `npm install`,
`npm run tauri -- build --no-bundle` → `target/release/libretune-app.exe`.

## Using it with HaniCore

Connection → **K-line** → cable connected to **ECU K-line pin** (bench) or
**car OBD socket** → COM port → 57600 or 124800 → Connect; import the board's
`.ini` from the HaniCore release when asked for the ECU definition.
