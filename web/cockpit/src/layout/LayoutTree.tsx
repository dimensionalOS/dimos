// Recursive renderer of the manifest's layout tree: a string leaf is a
// panel (component looked up by kind, UnknownPanel fallback), row/col nodes
// become flex containers whose children grow by their share (absent shares =
// equal split). A manifest without a layout falls back to one row of all
// non-page panels in manifest order, so manifest-less/auto bridges still
// render.

import type { LayoutNode, Manifest, PanelSpec } from "@dimos/shared/manifest";
import { getPanel, UnknownPanel } from "../panels/registry.tsx";
import type { ChannelStore } from "../session/store.ts";
import styles from "./LayoutTree.module.css";

function Node({ node, byId, store }: {
  node: LayoutNode;
  byId: Map<string, PanelSpec>;
  store: ChannelStore;
}) {
  if (typeof node === "string") {
    const spec = byId.get(node);
    if (spec === undefined) return null; // unreachable post-validation
    const Component = getPanel(spec.kind) ?? UnknownPanel;
    return <Component spec={spec} store={store} />;
  }
  const children = "row" in node ? node.row : node.col;
  return (
    <div className={"row" in node ? styles.row : styles.col}>
      {children.map((child, i) => (
        <div key={i} className={styles.cell} style={{ flexGrow: node.shares?.[i] ?? 1 }}>
          <Node node={child} byId={byId} store={store} />
        </div>
      ))}
    </div>
  );
}

export function LayoutTree({ manifest, store }: { manifest: Manifest; store: ChannelStore }) {
  const byId = new Map(manifest.panels.map((p) => [p.id, p]));
  const pages = new Set(manifest.pages);
  const gridIds = manifest.panels.filter((p) => !pages.has(p.id)).map((p) => p.id);
  const node = manifest.layout ?? (gridIds.length > 0 ? { row: gridIds } : null);
  if (node === null) return null;
  return (
    <div className={styles.root}>
      <Node node={node} byId={byId} store={store} />
    </div>
  );
}
