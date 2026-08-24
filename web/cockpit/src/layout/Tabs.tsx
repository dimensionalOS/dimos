// Page tabs: tab 0 is the grid (children), then one tab per manifest.pages
// entry, rendered full-page. With no pages (all of T7) no tab bar renders at
// all. Only the active tab's content mounts: a CSS-hidden canvas would keep
// decoding, since the sinks check document visibility, not element
// visibility. The active index is component state, so an epoch remount (a
// changed manifest) resets to the grid on purpose.

import { type ReactNode, useState } from "react";
import type { Manifest } from "@dimos/shared/manifest";
import { getPanel, UnknownPanel } from "../panels/registry.tsx";
import type { ChannelStore } from "../session/store.ts";
import styles from "./Tabs.module.css";

export function Tabs({ manifest, store, children }: {
  manifest: Manifest;
  store: ChannelStore;
  children: ReactNode;
}) {
  const [active, setActive] = useState(0);
  if (manifest.pages.length === 0) return <>{children}</>;
  const byId = new Map(manifest.panels.map((p) => [p.id, p]));
  let page: ReactNode = null;
  if (active > 0) {
    const spec = byId.get(manifest.pages[active - 1]);
    if (spec !== undefined) {
      const Component = getPanel(spec.kind) ?? UnknownPanel;
      page = <Component spec={spec} store={store} />;
    }
  }
  return (
    <>
      <div role="tablist" className={styles.bar}>
        <button
          type="button"
          role="tab"
          aria-selected={active === 0}
          className={active === 0 ? styles.tabActive : styles.tab}
          data-testid="tab-overview"
          onClick={() => setActive(0)}
        >
          overview
        </button>
        {manifest.pages.map((id, i) => {
          const spec = byId.get(id);
          const label = spec === undefined || spec.title === "" ? id : spec.title;
          return (
            <button
              key={id}
              type="button"
              role="tab"
              aria-selected={active === i + 1}
              className={active === i + 1 ? styles.tabActive : styles.tab}
              data-testid={`tab-${id}`}
              onClick={() => setActive(i + 1)}
            >
              {label}
            </button>
          );
        })}
      </div>
      <div role="tabpanel" className={styles.page}>
        {active === 0 ? children : page}
      </div>
    </>
  );
}
