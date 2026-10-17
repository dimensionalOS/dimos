// Recursive renderer of the manifest's layout tree: a string leaf is a
// panel (component looked up by kind, UnknownPanel fallback), row/col nodes
// become flex containers whose children grow by their share (absent shares =
// equal split). A manifest without a layout falls back to one row of all
// non-page panels in manifest order, so manifest-less/auto bridges still
// render.

import type { LayoutNode, Manifest, PanelSpec } from "@dimos/shared/manifest";
import { getPanel, type PanelProps, UnknownPanel } from "../panels/registry.tsx";
import styles from "./LayoutTree.module.css";

type PanelContext = Omit<PanelProps, "spec">;

/** The grid's tree: the authored layout, else one row of every non-page
 * panel, null when there is nothing to lay out. */
function gridNode(manifest: Manifest): LayoutNode | null {
  const pages = new Set(manifest.pages);
  const gridIds = manifest.panels.filter((p) => !pages.has(p.id)).map((p) => p.id);
  return manifest.layout ?? (gridIds.length > 0 ? { row: gridIds } : null);
}

function leaves(node: LayoutNode): string[] {
  if (typeof node === "string") return [node];
  return ("row" in node ? node.row : node.col).flatMap(leaves);
}

/** The panels the grid mounts, in tree order (what App reports as shown). */
export function gridPanelIds(manifest: Manifest): string[] {
  const node = gridNode(manifest);
  return node === null ? [] : leaves(node);
}

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
      {children.map((child, i) => (
        <div key={i} className={styles.cell} style={{ flexGrow: node.shares?.[i] ?? 1 }}>
          <Node node={child} byId={byId} panelProps={panelProps} />
        </div>
      ))}
    </div>
  );
}

export function LayoutTree({ manifest, ...panelProps }: { manifest: Manifest } & PanelContext) {
  const node = gridNode(manifest);
  if (node === null) return null;
  const byId = new Map(manifest.panels.map((p) => [p.id, p]));
  return (
    <div className={styles.root}>
      <Node node={node} byId={byId} panelProps={panelProps} />
    </div>
  );
}
