// Manifest domain model shared by the relay and the Cockpit. Mirrored in
// Python at dimos/web/relay_bridge/manifest.py and pinned by the golden
// vectors in ./fixtures/manifests.json (tested from both deno test and
// pytest).
//
// The transport (protocol.ts) checks only field shapes; this module owns the
// domain rules: bounded unique ids, positive rates, and panel/layout
// references that resolve. Panels and layout are minimal until T7 (the
// layout is a flat panel-id order, not a tree).

export type Delivery = "latest" | "reliable";

// One robot->viewer stream: encoding names the payload format (e.g. jpeg.v1,
// pose.json.v1), delivery picks the relay's forwarding policy, maxHz is the
// bridge's advertised send cap (informational for viewers).
export interface ChannelSpec {
  ch: string;
  encoding: string;
  delivery: Delivery;
  maxHz: number;
}

// Minimal until T7: kind is the panel-component registry key; channels lists
// the channel ids the panel consumes. Kind-specific config arrives with the
// panel system.
export interface PanelSpec {
  id: string;
  kind: string;
  channels: string[];
}

export interface Manifest {
  channels: ChannelSpec[];
  panels: PanelSpec[];
  /** Panel ids in display order (a layout tree replaces this in T7). */
  layout: string[];
}

/** Bound for channel/panel ids, encodings, and panel kinds. */
export const MAX_MANIFEST_ID_LEN = 64;

export class ManifestError extends Error {
  constructor(readonly code: string, message: string) {
    super(`${code}: ${message}`);
    this.name = "ManifestError";
  }
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

export function isChannelSpec(value: unknown): value is ChannelSpec {
  return (
    isRecord(value) &&
    typeof value.ch === "string" &&
    typeof value.encoding === "string" &&
    (value.delivery === "latest" || value.delivery === "reliable") &&
    typeof value.maxHz === "number"
  );
}

function isPanelSpec(value: unknown): value is PanelSpec {
  return (
    isRecord(value) &&
    typeof value.id === "string" &&
    typeof value.kind === "string" &&
    Array.isArray(value.channels) &&
    value.channels.every((ch) => typeof ch === "string")
  );
}

function boundedId(s: string): boolean {
  return s.length >= 1 && s.length <= MAX_MANIFEST_ID_LEN;
}

/**
 * Validated manifest from parsed JSON (or any untrusted value); throws
 * ManifestError. Absent panels/layout normalize to empty; unknown keys are
 * dropped (forward compatibility with newer bridges). Mirrors
 * parse_manifest() in manifest.py: shape first, then domain rules in
 * document order, so both sides report the same code (pinned by fixtures).
 */
export function parseManifest(value: unknown): Manifest {
  if (!isRecord(value)) throw new ManifestError("invalid_shape", "manifest must be an object");
  const rawChannels = value.channels;
  const rawPanels = value.panels === undefined ? [] : value.panels;
  const rawLayout = value.layout === undefined ? [] : value.layout;
  if (
    !Array.isArray(rawChannels) || !rawChannels.every(isChannelSpec) ||
    !Array.isArray(rawPanels) || !rawPanels.every(isPanelSpec) ||
    !Array.isArray(rawLayout) || !rawLayout.every((id) => typeof id === "string")
  ) {
    throw new ManifestError("invalid_shape", "bad channels/panels/layout shape");
  }
  const channels = rawChannels as ChannelSpec[];
  const panels = rawPanels as PanelSpec[];
  const layout = rawLayout as string[];

  const chIds = new Set<string>();
  for (const spec of channels) {
    if (!boundedId(spec.ch)) {
      throw new ManifestError(
        "invalid_channel_id",
        `channel id must be 1..${MAX_MANIFEST_ID_LEN} chars`,
      );
    }
    if (chIds.has(spec.ch)) {
      throw new ManifestError("duplicate_channel_id", `duplicate channel ${spec.ch}`);
    }
    chIds.add(spec.ch);
    if (!boundedId(spec.encoding)) {
      throw new ManifestError(
        "invalid_encoding",
        `encoding must be 1..${MAX_MANIFEST_ID_LEN} chars`,
      );
    }
    // isFinite also bounds maxHz to float64: JSON.parse turns an
    // out-of-range literal into Infinity. manifest.py mirrors this with an
    // explicit float64 bound (Python parses such integers exactly).
    if (!Number.isFinite(spec.maxHz) || spec.maxHz <= 0) {
      throw new ManifestError("invalid_max_hz", `maxHz for ${spec.ch} must be a positive number`);
    }
  }

  const panelIds = new Set<string>();
  for (const panel of panels) {
    if (!boundedId(panel.id) || !boundedId(panel.kind)) {
      throw new ManifestError(
        "invalid_panel",
        `panel id/kind must be 1..${MAX_MANIFEST_ID_LEN} chars`,
      );
    }
    if (panelIds.has(panel.id)) {
      throw new ManifestError("duplicate_panel_id", `duplicate panel ${panel.id}`);
    }
    panelIds.add(panel.id);
    for (const ch of panel.channels) {
      if (!chIds.has(ch)) {
        throw new ManifestError(
          "unknown_panel_channel",
          `panel ${panel.id} wants undeclared channel ${ch}`,
        );
      }
    }
  }

  for (const id of layout) {
    if (!panelIds.has(id)) {
      throw new ManifestError("unknown_layout_panel", `layout names unknown panel ${id}`);
    }
  }

  return {
    channels: channels.map((spec) => ({
      ch: spec.ch,
      encoding: spec.encoding,
      delivery: spec.delivery,
      maxHz: spec.maxHz,
    })),
    panels: panels.map((panel) => ({
      id: panel.id,
      kind: panel.kind,
      channels: [...panel.channels],
    })),
    layout: [...layout],
  };
}
