// Panel-component registry: manifest panel kinds -> React components, fixed
// at build time (the hosted cockpit cannot load code at runtime; adding
// frontend capability means adding entries here).
//
// getPanel deliberately returns undefined for unknown kinds: the
// subscription gate (channelSubscribable in session.ts) relies on that to
// avoid subscribing channels only a newer build could render. The renderers
// (LayoutTree/Tabs) fall back to UnknownPanel instead - forward
// compatibility with newer bridges without the traffic.

import type { ComponentType } from "react";
import type { PanelSpec } from "@dimos/shared";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import type { ChannelStore } from "../session/store.ts";
import { MapPanel } from "./MapPanel.tsx";
import styles from "./registry.module.css";
import { VideoPanel } from "./VideoPanel.tsx";

export interface PanelProps {
  spec: PanelSpec;
  store: ChannelStore;
}

const registry = new Map<string, ComponentType<PanelProps>>();

export function registerPanel(kind: string, component: ComponentType<PanelProps>): void {
  registry.set(kind, component);
}

export function getPanel(kind: string): ComponentType<PanelProps> | undefined {
  return registry.get(kind);
}

/** Render-only fallback for manifest panel kinds this build does not know.
 * Binds no channels and causes no subscriptions (see getPanel above). */
export function UnknownPanel({ spec }: PanelProps) {
  return (
    <PanelFrame spec={spec}>
      <span className={styles.unknown}>unknown panel kind {spec.kind}</span>
    </PanelFrame>
  );
}

registerPanel("video", VideoPanel);
registerPanel("map2d", MapPanel);
