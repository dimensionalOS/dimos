// Manifest domain model shared by the relay and the Cockpit. Mirrored in
// Python at dimos/web/relay_bridge/manifest.py and pinned by the golden
// vectors in ./fixtures/manifests.json (tested from both deno test and
// pytest).
//
// The transport (protocol.ts) carries the manifest as one opaque record;
// this module is the single owner of its structure and domain rules:
// version gate, bounded unique ids, positive rates, panel/layout/pages
// references that resolve, and kind-specific panel rules (video, map2d).
//
// Manifest v1 is frozen. Additive changes (new panel kinds, new params)
// ride the existing shape: unknown keys and kinds pass through validation.
// Breaking changes bump `version`, and the Cockpit refuses politely
// (unsupported_version).

export type Delivery = "latest" | "reliable";
export type Dir = "rx" | "tx";
// Generic-publish policy for a tx channel: "none" (default; specialized
// protocol paths like teleop only), "shared" (any authorized viewer may
// publish), "exclusive" (requires the per-robot publisher lease, W8).
// Transport delivery and publish authorization are independent: reliable
// does not imply publishable.
export type Publish = "none" | "shared" | "exclusive";

// One robot<->viewer stream: dir is the flow direction seen from the viewer
// (rx = robot->viewer; tx arrives with teleop/chat), encoding names the
// payload format (e.g. jpeg.v1, pose.json.v1), delivery picks the relay's
// forwarding policy, maxHz is the bridge's advertised send cap (for a
// publish channel: the aggregate accepted publish rate), params are encoder
// settings (informational for viewers). publish/requiredScope gate generic
// publishing; requiredScope names the operator scope a remote relay demands
// (null = none; the local relay never checks scopes).
export interface ChannelSpec {
  ch: string;
  dir: Dir;
  encoding: string;
  delivery: Delivery;
  maxHz: number;
  params: Record<string, unknown>;
  publish: Publish;
  requiredScope: string | null;
}

// kind is the panel-component registry key; channels lists the channel ids
// the panel consumes in kind-defined slot order (e.g. map2d: costmap, then
// the optional pose overlay). title "" means untitled (the frame falls back
// to the panel id).
export interface PanelSpec {
  id: string;
  kind: string;
  title: string;
  channels: string[];
  params: Record<string, unknown>;
}

// Recursive layout tree: a leaf is a panel id; row/col split the space with
// flex shares (absent shares = equal split, a renderer default that is
// deliberately not baked into normalized data).
export interface RowNode {
  row: LayoutNode[];
  shares?: number[];
}
export interface ColNode {
  col: LayoutNode[];
  shares?: number[];
}
export type LayoutNode = string | RowNode | ColNode;

export interface Manifest {
  version: 1;
  channels: ChannelSpec[];
  panels: PanelSpec[];
  /** null = no layout authored; the Cockpit falls back to a simple row. */
  layout: LayoutNode | null;
  /** Panel ids rendered as full-page tabs instead of grid cells. */
  pages: string[];
}

/** The only manifest version this build understands. */
export const MANIFEST_VERSION = 1;

/** Bound for channel/panel ids, encodings, and panel kinds. */
export const MAX_MANIFEST_ID_LEN = 64;

/** Channel ids with this prefix belong to protocol control (e.g. @control)
 * and can never be declared by a manifest. */
export const RESERVED_CHANNEL_PREFIX = "@";

export class ManifestError extends Error {
  constructor(readonly code: string, message: string) {
    super(`${code}: ${message}`);
    this.name = "ManifestError";
  }
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

// Raw (pre-normalization) spec shapes: dir/params/title may be absent.
// requiredScope also accepts explicit null so a normalized manifest parses
// idempotently (parse(parse(m)) == parse(m)); publish does not (null is a
// shape error, like dir).
interface RawChannelSpec {
  ch: string;
  dir?: Dir;
  encoding: string;
  delivery: Delivery;
  maxHz: number;
  params?: Record<string, unknown>;
  publish?: Publish;
  requiredScope?: string | null;
}

interface RawPanelSpec {
  id: string;
  kind: string;
  title?: string;
  channels: string[];
  params?: Record<string, unknown>;
}

function isChannelSpec(value: unknown): value is RawChannelSpec {
  return (
    isRecord(value) &&
    typeof value.ch === "string" &&
    (value.dir === undefined || value.dir === "rx" || value.dir === "tx") &&
    typeof value.encoding === "string" &&
    (value.delivery === "latest" || value.delivery === "reliable") &&
    typeof value.maxHz === "number" &&
    (value.params === undefined || isRecord(value.params)) &&
    (value.publish === undefined || value.publish === "none" || value.publish === "shared" ||
      value.publish === "exclusive") &&
    (value.requiredScope === undefined || value.requiredScope === null ||
      typeof value.requiredScope === "string")
  );
}

function isPanelSpec(value: unknown): value is RawPanelSpec {
  return (
    isRecord(value) &&
    typeof value.id === "string" &&
    typeof value.kind === "string" &&
    (value.title === undefined || typeof value.title === "string") &&
    Array.isArray(value.channels) &&
    value.channels.every((ch) => typeof ch === "string") &&
    (value.params === undefined || isRecord(value.params))
  );
}

function boundedId(s: string): boolean {
  return s.length >= 1 && s.length <= MAX_MANIFEST_ID_LEN;
}

function dirOf(spec: RawChannelSpec): Dir {
  return spec.dir ?? "rx";
}

function publishOf(spec: RawChannelSpec): Publish {
  return spec.publish ?? "none";
}

function scopeOf(spec: RawChannelSpec): string | null {
  return spec.requiredScope ?? null;
}

// "Supported JSON encoding" for generic publish is the family-name rule
// (json.v1, text.json.v1, ...): the manifest layer cannot see the codec
// registries, so real decodability is enforced at authoring time.
const JSON_ENCODING_RE = /(^|\.)json\.v[0-9]+$/;

/**
 * Depth-first layout validation + rebuild. A node's own structure (row/col
 * exclusivity, children, shares) is checked before its children; string
 * leaves check unknown before duplicate. `seen` is shared with the pages
 * loop so one panel can be placed only once across the whole cockpit.
 * Unknown node keys are dropped by the rebuild (global unknown-key policy).
 */
function validateLayout(node: unknown, panelIds: Set<string>, seen: Set<string>): LayoutNode {
  if (typeof node === "string") {
    if (!panelIds.has(node)) {
      throw new ManifestError("unknown_layout_panel", `layout names unknown panel ${node}`);
    }
    if (seen.has(node)) {
      throw new ManifestError("duplicate_layout_panel", `panel ${node} placed more than once`);
    }
    seen.add(node);
    return node;
  }
  if (!isRecord(node)) {
    throw new ManifestError("invalid_layout", "layout node must be a panel id or a row/col object");
  }
  const isRow = "row" in node;
  if (isRow === "col" in node) {
    throw new ManifestError("invalid_layout", "layout node needs exactly one of row/col");
  }
  const children = isRow ? node.row : node.col;
  if (!Array.isArray(children) || children.length === 0) {
    throw new ManifestError("invalid_layout", "row/col children must be a non-empty array");
  }
  let shares: number[] | undefined;
  if (node.shares !== undefined) {
    const raw = node.shares;
    // typeof excludes booleans; isFinite bounds to float64 like maxHz.
    if (
      !Array.isArray(raw) ||
      raw.length !== children.length ||
      !raw.every((s) => typeof s === "number" && Number.isFinite(s) && s > 0)
    ) {
      throw new ManifestError(
        "invalid_layout",
        "shares must be positive numbers, one per child",
      );
    }
    shares = [...(raw as number[])];
  }
  const rebuilt = children.map((child) => validateLayout(child, panelIds, seen));
  if (isRow) return shares === undefined ? { row: rebuilt } : { row: rebuilt, shares };
  return shares === undefined ? { col: rebuilt } : { col: rebuilt, shares };
}

/**
 * Validated manifest from parsed JSON (or any untrusted value); throws
 * ManifestError. Absent dir/params/title/layout/pages normalize to
 * "rx"/{}/""/null/[]; unknown keys are dropped (forward compatibility with
 * newer bridges). Mirrors parse_manifest() in manifest.py: shape, then
 * domain rules in document order, so both sides report the same code
 * (pinned by fixtures). The version gate runs before the channel/panel
 * shape checks so a structurally-alien future manifest still reports
 * unsupported_version, which the Cockpit turns into a polite notice.
 */
export function parseManifest(value: unknown): Manifest {
  if (!isRecord(value)) throw new ManifestError("invalid_shape", "manifest must be an object");
  if (typeof value.version !== "number") {
    throw new ManifestError("invalid_shape", "manifest version must be a number");
  }
  if (value.version !== MANIFEST_VERSION) {
    throw new ManifestError(
      "unsupported_version",
      `manifest version ${value.version} is not supported`,
    );
  }
  const rawChannels = value.channels;
  const rawPanels = value.panels === undefined ? [] : value.panels;
  if (
    !Array.isArray(rawChannels) || !rawChannels.every(isChannelSpec) ||
    !Array.isArray(rawPanels) || !rawPanels.every(isPanelSpec)
  ) {
    throw new ManifestError("invalid_shape", "bad channels/panels shape");
  }
  const channels = rawChannels as RawChannelSpec[];
  const panels = rawPanels as RawPanelSpec[];

  const chIds = new Map<string, RawChannelSpec>();
  for (const spec of channels) {
    if (!boundedId(spec.ch)) {
      throw new ManifestError(
        "invalid_channel_id",
        `channel id must be 1..${MAX_MANIFEST_ID_LEN} chars`,
      );
    }
    if (spec.ch.startsWith(RESERVED_CHANNEL_PREFIX)) {
      throw new ManifestError(
        "reserved_channel_id",
        `channel ids beginning with ${RESERVED_CHANNEL_PREFIX} are reserved for protocol control`,
      );
    }
    if (chIds.has(spec.ch)) {
      throw new ManifestError("duplicate_channel_id", `duplicate channel ${spec.ch}`);
    }
    chIds.set(spec.ch, spec);
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
    // Generic-publish rules. The manifest layer accepts "exclusive" (only
    // authoring and the bridge reject it until the lease ticket, W8).
    const publish = publishOf(spec);
    const scope = scopeOf(spec);
    if (dirOf(spec) === "rx" && publish !== "none") {
      throw new ManifestError("invalid_publish", `rx channel ${spec.ch} cannot declare publish`);
    }
    if (publish !== "none" && spec.delivery !== "reliable") {
      throw new ManifestError("invalid_publish", `publish channel ${spec.ch} must be reliable`);
    }
    if (publish !== "none" && !JSON_ENCODING_RE.test(spec.encoding)) {
      throw new ManifestError(
        "invalid_publish",
        `publish channel ${spec.ch} needs a JSON encoding`,
      );
    }
    if (scope !== null && publish === "none") {
      throw new ManifestError(
        "invalid_scope",
        `channel ${spec.ch} scope requires a publish policy`,
      );
    }
    if (scope !== null && !boundedId(scope)) {
      throw new ManifestError(
        "invalid_scope",
        `scope for ${spec.ch} must be 1..${MAX_MANIFEST_ID_LEN} chars`,
      );
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
    // Kind-specific rules (incl. dir=rx for viewer-rendered channels);
    // unknown kinds stay unvalidated (forward compatibility with newer
    // bridges).
    if (panel.kind === "video") {
      if (panel.channels.length !== 1) {
        throw new ManifestError(
          "invalid_video_panel",
          `video panel ${panel.id} must bind exactly one channel`,
        );
      }
      const bound = chIds.get(panel.channels[0])!;
      if (bound.encoding !== "jpeg.v1" || bound.delivery !== "latest" || dirOf(bound) !== "rx") {
        throw new ManifestError(
          "invalid_video_panel",
          `video panel ${panel.id} needs a jpeg.v1 latest rx channel`,
        );
      }
    }
    if (panel.kind === "map2d") {
      // channels[0] is the costmap; channels[1] (optional) the pose overlay.
      if (panel.channels.length !== 1 && panel.channels.length !== 2) {
        throw new ManifestError(
          "invalid_map2d_panel",
          `map2d panel ${panel.id} must bind one or two channels`,
        );
      }
      const costmap = chIds.get(panel.channels[0])!;
      if (
        costmap.encoding !== "costmap.zlib.v1" || costmap.delivery !== "latest" ||
        dirOf(costmap) !== "rx"
      ) {
        throw new ManifestError(
          "invalid_map2d_panel",
          `map2d panel ${panel.id} needs a costmap.zlib.v1 latest rx channel first`,
        );
      }
      if (panel.channels.length === 2) {
        const pose = chIds.get(panel.channels[1])!;
        if (pose.encoding !== "pose.json.v1" || dirOf(pose) !== "rx") {
          throw new ManifestError(
            "invalid_map2d_panel",
            `map2d panel ${panel.id} pose channel must be a pose.json.v1 rx channel`,
          );
        }
      }
    }
    if (panel.kind === "teleop") {
      if (panel.channels.length !== 1) {
        throw new ManifestError(
          "invalid_teleop_panel",
          `teleop panel ${panel.id} must bind exactly one channel`,
        );
      }
      const cmd = chIds.get(panel.channels[0])!;
      if (cmd.encoding !== "twist.json.v1" || cmd.delivery !== "latest" || dirOf(cmd) !== "tx") {
        throw new ManifestError(
          "invalid_teleop_panel",
          `teleop panel ${panel.id} needs a twist.json.v1 latest tx channel`,
        );
      }
    }
    if (panel.kind === "chat") {
      // channels: the text input (publish tx), the messages, the idle flag.
      if (panel.channels.length !== 3) {
        throw new ManifestError(
          "invalid_chat_panel",
          `chat panel ${panel.id} must bind three channels`,
        );
      }
      const [text, messages, idle] = panel.channels.map((ch) => chIds.get(ch)!);
      if (
        text.encoding !== "text.json.v1" || text.delivery !== "reliable" ||
        dirOf(text) !== "tx" || publishOf(text) !== "shared"
      ) {
        throw new ManifestError(
          "invalid_chat_panel",
          `chat panel ${panel.id} needs a text.json.v1 reliable shared tx channel first`,
        );
      }
      if (
        messages.encoding !== "chat.json.v1" || messages.delivery !== "reliable" ||
        dirOf(messages) !== "rx"
      ) {
        throw new ManifestError(
          "invalid_chat_panel",
          `chat panel ${panel.id} needs a chat.json.v1 reliable rx channel second`,
        );
      }
      if (idle.encoding !== "json.v1" || idle.delivery !== "latest" || dirOf(idle) !== "rx") {
        throw new ManifestError(
          "invalid_chat_panel",
          `chat panel ${panel.id} needs a json.v1 latest rx channel third`,
        );
      }
    }
  }

  const seen = new Set<string>();
  let layout: LayoutNode | null = null;
  if (value.layout !== undefined && value.layout !== null) {
    layout = validateLayout(value.layout, panelIds, seen);
  }

  const rawPages = value.pages === undefined ? [] : value.pages;
  if (!Array.isArray(rawPages) || !rawPages.every((id) => typeof id === "string")) {
    throw new ManifestError("invalid_shape", "bad pages shape");
  }
  const pages = rawPages as string[];
  for (const id of pages) {
    if (!panelIds.has(id)) {
      throw new ManifestError("unknown_page_panel", `pages names unknown panel ${id}`);
    }
    if (seen.has(id)) {
      throw new ManifestError("duplicate_layout_panel", `panel ${id} placed more than once`);
    }
    seen.add(id);
  }

  return {
    version: MANIFEST_VERSION,
    channels: channels.map((spec) => ({
      ch: spec.ch,
      dir: dirOf(spec),
      encoding: spec.encoding,
      delivery: spec.delivery,
      maxHz: spec.maxHz,
      params: { ...(spec.params ?? {}) },
      publish: publishOf(spec),
      requiredScope: scopeOf(spec),
    })),
    panels: panels.map((panel) => ({
      id: panel.id,
      kind: panel.kind,
      title: panel.title ?? "",
      channels: [...panel.channels],
      params: { ...(panel.params ?? {}) },
    })),
    layout,
    pages: [...pages],
  };
}
