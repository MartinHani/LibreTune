// Modified 2026-10-30/31 in the HaniCore K-line fork of LibreTune (see FORK.md).
/**
 * State model for the "Connect ECU" wizard (Phase 1 — navigation skeleton).
 *
 * The wizard guides a first-time ECU connection: pick a transport, enter the
 * connection parameters, connect and read the ECU's firmware signature, resolve
 * the matching INI definition (local → online → manual), then name and create
 * the project. This module holds only the *step ordering* logic so it can be
 * unit-tested without any UI or Tauri backend. Later phases fill each step in.
 */

/** How the user connects to the ECU. */
export type WizardTransport = "usb" | "bluetooth" | "kline" | "wifi" | "offline";

/** The ordered steps of the wizard. */
export type WizardStep = "transport" | "params" | "connect" | "resolveIni" | "name";

/**
 * The steps that apply for a given transport.
 *
 * The "offline" path skips everything connection-related and goes straight from
 * choosing the transport to naming the project (the user picks an INI by hand,
 * matching today's New Project behaviour). A live transport walks the full
 * connect → detect → resolve flow.
 */
export function wizardSteps(transport: WizardTransport | null): WizardStep[] {
  if (transport === "offline") return ["transport", "name"];
  return ["transport", "params", "connect", "resolveIni", "name"];
}

/** Whether a transport talks over a serial port (USB, a Bluetooth COM port, or a K-line cable). */
export function isSerialTransport(transport: WizardTransport | null): boolean {
  return transport === "usb" || transport === "bluetooth" || transport === "kline";
}

/** Common baud rates offered for serial connections. */
export const WIZARD_BAUD_RATES = [9600, 38400, 57600, 115200, 230400, 460800, 921600];

/** Rates a K-line ECU can be asked for (direct, or negotiated through KWP). */
export const WIZARD_KLINE_BAUD_RATES = [57600, 115200, 124800];

/**
 * The backend connection type for a transport. A K-line cable is either on
 * the ECU's K-line pin, or in the car's OBD socket (KWP wake-up first).
 */
export function wizardConnectionType(
  transport: WizardTransport | null,
  klineObd: boolean,
): "Serial" | "Tcp" | "KLine" | "KLineKwp" {
  if (transport === "wifi") return "Tcp";
  if (transport === "kline") return klineObd ? "KLineKwp" : "KLine";
  return "Serial";
}

/** Connection parameters collected on the "params" step. */
export interface WizardConnectionParams {
  /** Serial / Bluetooth COM port. */
  port: string;
  /** Serial / Bluetooth baud rate. */
  baud: number;
  /** WiFi / TCP host. */
  host: string;
  /** WiFi / TCP port. */
  tcpPort: number;
}

/**
 * Whether the connection parameters for a transport are complete enough to
 * attempt a connection. Serial/Bluetooth need a port; WiFi needs host + port;
 * offline needs nothing.
 */
export function paramsComplete(
  transport: WizardTransport | null,
  p: WizardConnectionParams,
): boolean {
  if (isSerialTransport(transport)) return p.port.trim().length > 0;
  if (transport === "wifi") return p.host.trim().length > 0 && p.tcpPort > 0;
  return true;
}

/** A local INI candidate matched against the ECU signature. */
export interface WizardIniMatch {
  /** Repository ID, usable directly with `create_project`. */
  id: string;
  path: string;
  name: string;
  signature: string;
  match_type: "exact" | "partial" | "mismatch";
}

/**
 * Pick the best local INI for a signature: an exact match wins, otherwise the
 * first partial match. Full mismatches are never auto-selected.
 */
export function bestLocalMatch(matches: WizardIniMatch[]): WizardIniMatch | null {
  return (
    matches.find((m) => m.match_type === "exact") ??
    matches.find((m) => m.match_type === "partial") ??
    null
  );
}

/**
 * Clean a firmware signature read from an ECU.
 *
 * ECUs frequently pad the signature returned by the `Q`/`S` command with a
 * trailing NUL (or other control bytes), and TunerStudio/MSDroid strip these
 * before using it. If they leak through they corrupt both the `signature=`
 * comparison and the derived online URL (a trailing NUL turns a valid rusEFI
 * URL into a 400 Bad Request). We drop every ASCII control character
 * (code < 0x20 and 0x7F) and trim surrounding whitespace.
 */
export function sanitizeSignature(signature: string): string {
  // Drop ASCII control characters (code < 0x20 and 0x7F, e.g. a trailing
  // NUL the ECU pads the signature with) and trim, matching TunerStudio.
  return Array.from(signature)
    .filter((ch) => {
      const code = ch.charCodeAt(0);
      return code >= 0x20 && code !== 0x7f;
    })
    .join("")
    .trim();
}

/**
 * Derive the deterministic online `.ini` URL from a firmware signature, the way
 * rusEFI/FOME publish definitions (rusEFI wiki "INI lookup logic").
 *
 * The signature is lower-cased and every space and dot becomes a `/`, then
 * appended to the rusEFI online-INI base. Example:
 *   "rusEFI master.2026.08.17.mre_f4.2452009527"
 *   → https://rusefi.com/online/ini/rusefi/master/2026/08/17/mre_f4/2452009527.ini
 *
 * Returns `null` for signatures that don't use this scheme (e.g. Speeduino,
 * MegaSquirt), which are resolved by other means.
 */
export function deriveOnlineIniUrl(signature: string): string | null {
  const sig = sanitizeSignature(signature);
  const first = sig.split(/[ .]/)[0]?.toLowerCase() ?? "";
  if (first !== "rusefi" && first !== "fome") return null;
  const path = sig.toLowerCase().replace(/ /g, "/").replace(/\./g, "/");
  return `https://rusefi.com/online/ini/${path}.ini`;
}

/**
 * Speeduino publishes a single canonical `.ini` for the current release
 * rather than one per firmware build, so there's no per-signature path to
 * derive — any `speeduino …` signature resolves to the same definition.
 */
export function deriveSpeeduinoIniUrl(signature: string): string | null {
  const sig = sanitizeSignature(signature).toLowerCase();
  if (!sig.startsWith("speeduino")) return null;
  return "https://raw.githubusercontent.com/noisymime/speeduino/master/reference/speeduino.ini";
}

/** The step after `current`, or `current` itself when already at the last step. */
export function nextStep(current: WizardStep, transport: WizardTransport | null): WizardStep {
  const steps = wizardSteps(transport);
  const i = steps.indexOf(current);
  if (i < 0) return steps[0];
  return steps[Math.min(i + 1, steps.length - 1)];
}

/** The step before `current`, or `current` itself when already at the first step. */
export function prevStep(current: WizardStep, transport: WizardTransport | null): WizardStep {
  const steps = wizardSteps(transport);
  const i = steps.indexOf(current);
  if (i <= 0) return steps[0];
  return steps[i - 1];
}

/** Whether `current` is the final step for this transport. */
export function isLastStep(current: WizardStep, transport: WizardTransport | null): boolean {
  const steps = wizardSteps(transport);
  return steps.indexOf(current) === steps.length - 1;
}

/** Human labels for the transports. */
export function transportLabel(t: WizardTransport): string {
  switch (t) {
    case "usb":
      return "USB / Serial";
    case "bluetooth":
      return "Bluetooth";
    case "kline":
      return "K-line (KKL / VAG-COM cable)";
    case "wifi":
      return "WiFi / Network (TCP)";
    case "offline":
      return "No connection right now";
  }
}

/** Short title shown in the wizard header for each step. */
export function stepTitle(step: WizardStep): string {
  switch (step) {
    case "transport":
      return "How do you want to connect?";
    case "params":
      return "Connection settings";
    case "connect":
      return "Connecting & detecting ECU";
    case "resolveIni":
      return "ECU definition";
    case "name":
      return "Name your project";
  }
}
