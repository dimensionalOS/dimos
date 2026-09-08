// Recursive renderer of the manifest's layout tree: a string leaf is a
// panel (component looked up by kind, UnknownPanel fallback), row/col nodes
// become flex containers whose children grow by their share (absent shares =
// equal split; a strip kind sizes to its content instead and takes none). A
// manifest without a layout falls back to one row of all non-page panels in
// manifest order, so manifest-less/auto bridges still render.

import type { LayoutNode, Manifest, PanelSpec } from "@dimos/shared/manifest";
import { getPanel, type PanelProps, STRIP_KINDS, UnknownPanel } from "../panels/registry.tsx";
import styles from "./LayoutTree.module.css";

function isStrip(node: LayoutNode, byId: Map<string, PanelSpec>): boolean {
  return typeof node === "string" && STRIP_KINDS.has(byId.get(node)?.kind ?? "");
}

type PanelContext = Omit<PanelProps, "spec">;

function Node({ node, byId, panelProps }: {
  node: LayoutNode;
  byId: Map<string, PanelSpec>;
  panelProps: PanelContext;
}) {
  if (typeof node === "string") {
    const spec = byId.get(node);
    if (spec === undefined) return null; // unreachable post-validation
    const Component = getPanel(spec.kind) ?? UnknownPanel;
    return <Component spec={spec} {...panelProps} />;
  }
  const children = "row" in node ? node.row : node.col;
  return (
    <div className={"row" in node ? styles.row : styles.col}>
      {children.map((child, i) =>
        isStrip(child, byId)
          ? (
            <div key={i} className={styles.cellStrip}>
              <Node node={child} byId={byId} panelProps={panelProps} />
            </div>
          )
          : (
            <div key={i} className={styles.cell} style={{ flexGrow: node.shares?.[i] ?? 1 }}>
              <Node node={child} byId={byId} panelProps={panelProps} />
            </div>
          )
      )}
    </div>
  );
}

export function LayoutTree({ manifest, ...panelProps }: { manifest: Manifest } & PanelContext) {
  const byId = new Map(manifest.panels.map((p) => [p.id, p]));
  const pages = new Set(manifest.pages);
  const gridIds = manifest.panels.filter((p) => !pages.has(p.id)).map((p) => p.id);
  const node = manifest.layout ?? (gridIds.length > 0 ? { row: gridIds } : null);
  if (node === null) return null;
  return (
    <div className={styles.root}>
      <Node node={node} byId={byId} panelProps={panelProps} />
    </div>
  );
}
