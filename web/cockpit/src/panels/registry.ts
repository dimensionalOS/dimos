// Panel-component registry: manifest panel kinds -> React components, fixed
// at build time (the hosted cockpit cannot load code at runtime; adding
// frontend capability means adding entries here). Unknown kinds are skipped
// by the grid: forward compatibility with newer bridges.

import type { ComponentType } from "react";
import type { PanelSpec } from "@dimos/shared";
import type { ChannelStore } from "../session/store.ts";
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

registerPanel("video", VideoPanel);
