// Fixed layout v0: the manifest's panels in manifest order in a simple CSS
// grid. The T7 layout tree (rows/cols with shares, pages) replaces this.

import type { PanelSpec } from "@dimos/shared";
import type { ChannelStore } from "../session/store.ts";
import { getPanel } from "./registry.ts";
import styles from "./PanelGrid.module.css";

export function PanelGrid({ panels, store }: { panels: PanelSpec[]; store: ChannelStore }) {
  const known = panels.flatMap((spec) => {
    const Component = getPanel(spec.kind);
    // Unknown kind: this build has no such panel component (newer bridge);
    // the channel list below still shows the underlying channels.
    return Component === undefined ? [] : [{ spec, Component }];
  });
  if (known.length === 0) return null;
  return (
    <div className={styles.grid}>
      {known.map(({ spec, Component }) => <Component key={spec.id} spec={spec} store={store} />)}
    </div>
  );
}
