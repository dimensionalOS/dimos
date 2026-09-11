// The cockpit's subscription policy: which manifest channels this build puts
// to use, and the consumer that holds SDK subscription handles for them. The
// SDK itself subscribes nothing; the cockpit computes the set from its
// decoder registry and the panels on screen and reconciles on every adopted
// manifest and view change.

import type { ChannelSpec, DecoderRegistry, Session } from "@dimos/sdk";
import { createDecoderRegistry, isLcmSchema, TRACK_ENCODING } from "@dimos/sdk";
import type { PanelSpec } from "@dimos/shared";
import { getPanel } from "./panels/registry.tsx";

/** The cockpit's one decoder registry, passed to connect() and consulted by
 * the subscription gate and the raw channel table. */
export const cockpitDecoders = createDecoderRegistry();

// Encodings whose subscription costs real encode CPU and bandwidth (a track
// sub makes the robot encode H.264); subscribed only when a panel this build
// can render binds them.
const PANEL_ONLY_ENCODINGS = new Set(["jpeg.v1", "costmap.zlib.v1", TRACK_ENCODING]);

/** Channels only a panel may subscribe: the encodings above, and a *.lcm.v1
 * channel whose schema has a variable-length array (a point cloud, a scan, a
 * path): every frame costs the message's full size, far more than the channel
 * table's preview is worth. A bounded message (a pose, an odometry)
 * subscribes like json.v1 so the table can show it. */
function panelOnly(spec: ChannelSpec): boolean {
  if (PANEL_ONLY_ENCODINGS.has(spec.encoding)) return true;
  const lcm = spec.params.lcm;
  return isLcmSchema(lcm) &&
    Object.values(lcm.structs).some((rows) =>
      rows.some(([, , dims]) => dims !== null && dims.some((d) => typeof d === "string"))
    );
}

/** True when this build can put the channel to use: rx only, it has a
 * decoder, and a panel-only channel is additionally bound by a renderable
 * panel among `panels` (the manifest's for "usable at all", the ones on
 * screen for "held right now"). getPanel must keep returning undefined for
 * unknown kinds here: an UnknownPanel fallback in this gate would make it
 * vacuously true and subscribe every video/costmap channel of a newer bridge
 * (the render-only fallback lives in LayoutTree/Tabs). */
export function channelSubscribable(
  spec: ChannelSpec,
  panels: PanelSpec[],
  registry: DecoderRegistry = cockpitDecoders,
): boolean {
  if (spec.dir !== "rx") return false;
  if (registry.resolve(spec) === undefined) return false;
  if (!panelOnly(spec)) return true;
  return panels.some((p) => getPanel(p.kind) !== undefined && p.channels.includes(spec.ch));
}

/**
 * Channels worth subscribing: only rx channels with a decoder, and
 * panel-only channels only when a renderable panel binds them. Subscribing
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
 * Hold SDK subscription handles for the adopted manifest's subscribable
 * channels: cheap ones whatever is on screen, panel-only ones (video,
 * costmaps, bulk LCM) only while a panel binding them is shown, so another
 * page tab or the channels tab ends the pull and, once the last viewer is
 * gone, the robot's encoding. App reports the shown panels through
 * `setShownPanels`; the set is reconciled on that and on every status
 * change. While the manifest is null (robot gone, watch ambiguous) the
 * handles are kept: the relay keeps its viewer subscriptions across a robot
 * restart too, so a returning robot reattaches without unsub/sub churn.
 * `dispose` releases everything (tests; the page never disposes).
 */
export function installAutoSubscriptions(
  session: Pick<Session, "status" | "subscribe">,
  registry: DecoderRegistry = cockpitDecoders,
): { setShownPanels(ids: Iterable<string>): void; dispose(): void } {
  const held = new Map<string, () => void>();
  let shown = new Set<string>();
  const reconcile = () => {
    const manifest = session.status.get().manifest;
    if (manifest === null) return;
    const panels = manifest.panels.filter((p) => shown.has(p.id));
    const want = new Set(
      subscribableChannels(manifest.channels, panels, registry).map((s) => s.ch),
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
  return {
    setShownPanels(ids) {
      shown = new Set(ids);
      reconcile();
    },
    dispose() {
      unsubscribe();
      for (const release of held.values()) release();
      held.clear();
    },
  };
}
