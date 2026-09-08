// The cockpit's subscription policy: which manifest channels this build puts
// to use, and the consumer that holds SDK subscription handles for them. The
// SDK itself subscribes nothing; the cockpit computes the set from its
// decoder registry and its renderable panels and reconciles on every adopted
// manifest.

import type { ChannelSpec, DecoderRegistry, Session } from "@dimos/sdk";
import { createDecoderRegistry } from "@dimos/sdk";
import type { PanelSpec } from "@dimos/shared";
import { getPanel } from "./panels/registry.tsx";

/** The cockpit's one decoder registry, passed to connect() and consulted by
 * the subscription gate and the raw channel table. */
export const cockpitDecoders = createDecoderRegistry();

// Encodings whose subscription costs real encode CPU and bandwidth;
// subscribed only when a panel this build can render binds them.
const PANEL_ONLY_ENCODINGS = new Set(["jpeg.v1", "costmap.zlib.v1"]);

/** True when this build can put the channel to use: rx only, it has a
 * decoder, and a panel-only encoding is additionally bound by a renderable
 * panel. getPanel must keep returning undefined for unknown kinds here: an
 * UnknownPanel fallback in this gate would make it vacuously true and
 * subscribe every video/costmap channel of a newer bridge (the render-only
 * fallback lives in LayoutTree/Tabs). Page-tab visibility does not gate
 * subscriptions in T7 (deferred until a page panel carries real rate). */
export function channelSubscribable(
  spec: ChannelSpec,
  panels: PanelSpec[],
  registry: DecoderRegistry = cockpitDecoders,
): boolean {
  if (spec.dir !== "rx") return false;
  if (registry.get(spec.encoding) === undefined) return false;
  if (!PANEL_ONLY_ENCODINGS.has(spec.encoding)) return true;
  return panels.some((p) => getPanel(p.kind) !== undefined && p.channels.includes(spec.ch));
}

/**
 * Channels worth subscribing: only rx channels with a decoder, and
 * panel-only encodings only when a renderable panel binds them. Subscribing
 * to channels nobody can render wastes encode CPU and bandwidth, and a
 * high-rate JPEG stream nobody renders overflows the relay's reliable FIFO
 * under Firefox's tighter QUIC credit (the relay kicks the viewer every
 * ~8 s).
 */
export function subscribableChannels(
  channels: ChannelSpec[],
  panels: PanelSpec[],
  registry: DecoderRegistry = cockpitDecoders,
): ChannelSpec[] {
  return channels.filter((spec) => channelSubscribable(spec, panels, registry));
}

/**
 * Hold SDK subscription handles for every subscribable channel of the
 * adopted manifest, reconciling on each adoption. While the manifest is null
 * (robot gone, watch ambiguous) the handles are kept: the relay keeps its
 * viewer subscriptions across a robot restart too, so a returning robot
 * reattaches without unsub/sub churn. Returns a disposer that releases
 * everything (tests; the page never disposes).
 */
export function installAutoSubscriptions(
  session: Pick<Session, "status" | "subscribe">,
  registry: DecoderRegistry = cockpitDecoders,
): () => void {
  const held = new Map<string, () => void>();
  let applied: unknown = null;
  const reconcile = () => {
    const manifest = session.status.get().manifest;
    if (manifest === null || manifest === applied) return;
    applied = manifest;
    const want = new Set(
      subscribableChannels(manifest.channels, manifest.panels, registry).map((s) => s.ch),
    );
    for (const [ch, release] of held) {
      if (!want.has(ch)) {
        held.delete(ch);
        release();
      }
    }
    for (const ch of want) {
      if (!held.has(ch)) held.set(ch, session.subscribe(ch, () => {}));
    }
  };
  const unsubscribe = session.status.subscribe(reconcile);
  reconcile();
  return () => {
    unsubscribe();
    for (const release of held.values()) release();
    held.clear();
  };
}
