// RerunPanel — embeds the STOCK Rerun web viewer (WASM), fed live by the dimos
// rerun bridge's gRPC server (serve_grpc :9877). This is the "reuse Rerun for 3D"
// path: dimos topics → RerunBridgeModule → rr.log → serve_grpc → this viewer.
//
// Click-to-navigate without forking the viewer: the STOCK viewer surfaces the
// 3D click world-position to JS via the `selection_change` event (mapped by the
// react wrapper to the onSelectionChange prop). We forward it as a nav goal —
// exactly what the dimos-viewer fork does natively, in ~10 lines instead of a
// 4–6 day wasm-websocket port.
//
// Lives in a tab and stays MOUNTED when hidden (display:none) so its grpc stream
// keeps draining; on re-show we nudge a resize so it re-fits the container.
//
// Pin matters: @rerun-io/web-viewer-react must match rerun-sdk (0.32.0-alpha.1).
import { useEffect } from "react";
import WebViewer from "@rerun-io/web-viewer-react";
import { useDimosClient } from "../dimos";

const PROXY = import.meta.env.VITE_RERUN_PROXY ?? "rerun+http://127.0.0.1:9877/proxy";

export function RerunPanel({ active = true }: { active?: boolean }) {
  const client = useDimosClient();
  // Stays MOUNTED across tab switches (cold remount = a renderer-freezing grpc
  // re-stream), just hidden via display:none on 2D. Nudge a resize when it becomes
  // visible again so it re-fits the container after being hidden.
  useEffect(() => {
    if (active) requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
  }, [active]);
  return (
    <div
      className="panel"
      style={{ display: "flex", flexDirection: "column", flex: 1, minHeight: 0 }}
    >
      <div className="panel-title">
        Rerun · 3D (stock web viewer ← serve_grpc) · click a point → nav goal
      </div>
      {
        /* relative box gives a flex-bounded height; the absolute inset:0 child is
          removed from normal flow so the viewer's self-resizing canvas can never
          push its ancestors taller (was a per-frame runaway-growth feedback loop). */
      }
      <div style={{ flex: 1, minHeight: 0, position: "relative" }}>
        <div
          style={{
            position: "absolute",
            inset: 0,
            overflow: "hidden",
            borderRadius: 8,
            background: "#0b0e14",
          }}
        >
          <WebViewer
            rrd={PROXY}
            width="100%"
            height="100%"
            hide_welcome_screen
            onSelectionChange={(e: any) => {
              const item = e?.items?.find((i: any) => i?.type === "entity" && i?.position);
              if (item?.position) {
                const [x, y, z] = item.position;
                client?.navigate(x, y, z);
              }
            }}
          />
        </div>
      </div>
    </div>
  );
}
