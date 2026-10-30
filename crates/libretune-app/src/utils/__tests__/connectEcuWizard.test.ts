// Modified 2026-10-30/31 in the HaniCore K-line fork of LibreTune (see FORK.md).
import { describe, it, expect } from "vitest";
import {
  wizardSteps,
  nextStep,
  prevStep,
  isLastStep,
  transportLabel,
  stepTitle,
  isSerialTransport,
  paramsComplete,
  bestLocalMatch,
  deriveOnlineIniUrl,
  deriveSpeeduinoIniUrl,
  sanitizeSignature,
  wizardConnectionType,
  type WizardStep,
  type WizardIniMatch,
} from "../connectEcuWizard";

describe("wizardSteps", () => {
  it("walks the full flow for a live transport", () => {
    const expected: WizardStep[] = ["transport", "params", "connect", "resolveIni", "name"];
    expect(wizardSteps("usb")).toEqual(expected);
    expect(wizardSteps("bluetooth")).toEqual(expected);
    expect(wizardSteps("wifi")).toEqual(expected);
    expect(wizardSteps(null)).toEqual(expected);
  });

  it("skips connection steps when offline", () => {
    expect(wizardSteps("offline")).toEqual(["transport", "name"]);
  });
});

describe("nextStep / prevStep", () => {
  it("advances through a live flow and clamps at the end", () => {
    expect(nextStep("transport", "usb")).toBe("params");
    expect(nextStep("params", "usb")).toBe("connect");
    expect(nextStep("connect", "usb")).toBe("resolveIni");
    expect(nextStep("resolveIni", "usb")).toBe("name");
    expect(nextStep("name", "usb")).toBe("name"); // clamps
  });

  it("goes back and clamps at the start", () => {
    expect(prevStep("name", "usb")).toBe("resolveIni");
    expect(prevStep("params", "usb")).toBe("transport");
    expect(prevStep("transport", "usb")).toBe("transport"); // clamps
  });

  it("jumps straight to name (and back) on the offline path", () => {
    expect(nextStep("transport", "offline")).toBe("name");
    expect(prevStep("name", "offline")).toBe("transport");
  });
});

describe("isLastStep", () => {
  it("is true only on the final step of the active path", () => {
    expect(isLastStep("name", "usb")).toBe(true);
    expect(isLastStep("resolveIni", "usb")).toBe(false);
    expect(isLastStep("name", "offline")).toBe(true);
    expect(isLastStep("transport", "offline")).toBe(false);
  });
});

describe("isSerialTransport", () => {
  it("treats USB and Bluetooth as serial, WiFi and offline as not", () => {
    expect(isSerialTransport("usb")).toBe(true);
    expect(isSerialTransport("bluetooth")).toBe(true);
    expect(isSerialTransport("wifi")).toBe(false);
    expect(isSerialTransport("offline")).toBe(false);
  });

  it("treats a K-line cable as serial", () => {
    expect(isSerialTransport("kline")).toBe(true);
  });
});

describe("wizardConnectionType", () => {
  it("maps the transport (and the K-line cable's place) to the backend type", () => {
    expect(wizardConnectionType("usb", false)).toBe("Serial");
    expect(wizardConnectionType("wifi", false)).toBe("Tcp");
    expect(wizardConnectionType("kline", false)).toBe("KLine");
    expect(wizardConnectionType("kline", true)).toBe("KLineKwp");
    // the OBD choice only means something for a K-line cable
    expect(wizardConnectionType("usb", true)).toBe("Serial");
  });
});

describe("paramsComplete", () => {
  const base = { port: "", baud: 115200, host: "", tcpPort: 29000 };

  it("requires a port for serial transports", () => {
    expect(paramsComplete("usb", base)).toBe(false);
    expect(paramsComplete("usb", { ...base, port: "COM3" })).toBe(true);
    expect(paramsComplete("bluetooth", { ...base, port: "COM7" })).toBe(true);
  });

  it("requires host and a positive port for WiFi", () => {
    expect(paramsComplete("wifi", base)).toBe(false);
    expect(paramsComplete("wifi", { ...base, host: "192.168.1.10" })).toBe(true);
    expect(paramsComplete("wifi", { ...base, host: "192.168.1.10", tcpPort: 0 })).toBe(false);
  });

  it("needs nothing for offline", () => {
    expect(paramsComplete("offline", base)).toBe(true);
  });
});

describe("bestLocalMatch", () => {
  const m = (name: string, match_type: WizardIniMatch["match_type"]): WizardIniMatch => ({
    id: name,
    path: `/${name}.ini`,
    name,
    signature: name,
    match_type,
  });

  it("prefers an exact match over partials", () => {
    const matches = [m("a", "partial"), m("b", "exact"), m("c", "partial")];
    expect(bestLocalMatch(matches)?.name).toBe("b");
  });

  it("falls back to the first partial match", () => {
    expect(bestLocalMatch([m("a", "partial"), m("b", "partial")])?.name).toBe("a");
  });

  it("never auto-selects a full mismatch, and handles empty", () => {
    expect(bestLocalMatch([m("a", "mismatch")])).toBeNull();
    expect(bestLocalMatch([])).toBeNull();
  });
});

describe("deriveOnlineIniUrl", () => {
  it("derives the rusEFI URL from a signature (wiki example)", () => {
    expect(deriveOnlineIniUrl("rusEFI master.2026.08.17.mre_f4.2452009527")).toBe(
      "https://rusefi.com/online/ini/rusefi/master/2026/08/17/mre_f4/2452009527.ini",
    );
  });

  it("lowercases the leading token and handles FOME", () => {
    expect(deriveOnlineIniUrl("FOME master.2026.01.02.board")).toBe(
      "https://rusefi.com/online/ini/fome/master/2026/01/02/board.ini",
    );
  });

  it("returns null for non-rusEFI signatures", () => {
    expect(deriveOnlineIniUrl("speeduino 202409")).toBeNull();
    expect(deriveOnlineIniUrl("MS2Extra comms342aM")).toBeNull();
    expect(deriveOnlineIniUrl("")).toBeNull();
  });
});

describe("sanitizeSignature", () => {
  const NUL = String.fromCharCode(0);
  const CR = String.fromCharCode(13);
  const LF = String.fromCharCode(10);
  const US = String.fromCharCode(31);
  it("strips trailing NUL and control bytes and trims", () => {
    expect(sanitizeSignature("rusEFI master.2026.08.17.mre_f4.2452009527" + NUL)).toBe(
      "rusEFI master.2026.08.17.mre_f4.2452009527",
    );
    expect(sanitizeSignature("  speeduino 202409" + CR + LF)).toBe("speeduino 202409");
    expect(sanitizeSignature("a" + NUL + "b" + US + "c")).toBe("abc");
  });
});

describe("deriveOnlineIniUrl with a padded signature", () => {
  it("ignores a trailing NUL so the URL stays valid", () => {
    expect(deriveOnlineIniUrl("rusEFI master.2026.08.17.mre_f4.2452009527" + String.fromCharCode(0))).toBe(
      "https://rusefi.com/online/ini/rusefi/master/2026/08/17/mre_f4/2452009527.ini",
    );
  });
});

describe("deriveSpeeduinoIniUrl", () => {
  it("resolves any speeduino signature to the canonical definition", () => {
    expect(deriveSpeeduinoIniUrl("speeduino 202508")).toBe(
      "https://raw.githubusercontent.com/noisymime/speeduino/master/reference/speeduino.ini",
    );
    expect(deriveSpeeduinoIniUrl("Speeduino 202508" + String.fromCharCode(0))).toBe(
      "https://raw.githubusercontent.com/noisymime/speeduino/master/reference/speeduino.ini",
    );
  });

  it("returns null for non-Speeduino signatures", () => {
    expect(deriveSpeeduinoIniUrl("rusEFI master.2026.08.17.mre_f4.2452009527")).toBeNull();
    expect(deriveSpeeduinoIniUrl("")).toBeNull();
  });
});

describe("labels", () => {
  it("labels transports and steps", () => {
    expect(transportLabel("usb")).toMatch(/USB/);
    expect(transportLabel("offline")).toMatch(/No connection/);
    expect(stepTitle("resolveIni")).toMatch(/definition/i);
  });
});
