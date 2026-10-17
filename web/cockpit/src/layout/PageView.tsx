// The header's page tabs pick what <main> shows: null is the grid (children),
// any page id selects the matching manifest page, full-page. Only the
// active one mounts: a CSS-hidden canvas would keep decoding, since the sinks
// check document visibility, not element visibility. The active id is App
// state (above the epoch-keyed <main>), so a robot restart keeps the operator
// on their page; an id the new manifest no longer has falls back to the grid.

import type { ReactNode } from "react";
import type { Manifest } from "@dimos/shared/manifest";
import { getPanel, type PanelProps, UnknownPanel } from "../panels/registry.tsx";
import styles from "./PageView.module.css";

export interface PageTab {
  id: string;
  label: string;
}

/** The page tabs a manifest offers, in manifest order; label = title || id. */
export function pageTabs(manifest: Manifest): PageTab[] {
  const byId = new Map(manifest.panels.map((p) => [p.id, p]));
  return manifest.pages.map((id) => {
    const spec = byId.get(id);
    return { id, label: spec === undefined || spec.title === "" ? id : spec.title };
  });
}

export function PageView({ manifest, page, children, ...panelProps }: {
  manifest: Manifest;
  page: string | null;
  children: ReactNode;
} & Omit<PanelProps, "spec">) {
  const spec = page === null ? undefined : manifest.panels.find((p) => p.id === page);
  let content = children;
  if (spec !== undefined) {
    const Component = getPanel(spec.kind) ?? UnknownPanel;
    content = <Component spec={spec} {...panelProps} />;
  }
  return <div role="tabpanel" className={styles.page}>{content}</div>;
}
