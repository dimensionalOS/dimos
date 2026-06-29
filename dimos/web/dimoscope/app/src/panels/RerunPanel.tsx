// RerunPanel — embeds the STOCK Rerun web viewer (WASM), fed live by the dimos
// rerun bridge's gRPC server (serve_grpc :9877). This is the "reuse Rerun for 3D"
// path: dimos topics → RerunBridgeModule → rr.log → serve_grpc → this viewer.
//
// Pin matters: @rerun-io/web-viewer-react must match rerun-sdk (0.32.0-alpha.1).
// Run the bridge:  DIMOS_TRANSPORT=lcm python -c \
//   "from dimos.visualization.rerun.bridge import run_bridge; run_bridge(rerun_open='web', rerun_web=True)"
import WebViewer from "@rerun-io/web-viewer-react";

const PROXY = import.meta.env.VITE_RERUN_PROXY ?? "rerun+http://127.0.0.1:9877/proxy";

export function RerunPanel() {
  return (
    <div className="panel" style={{ flex: "1 1 640px", minWidth: 520 }}>
      <div className="panel-title">Rerun · 3D (stock web viewer ← serve_grpc)</div>
      <div style={{ height: 520, borderRadius: 8, overflow: "hidden", background: "#0b0e14" }}>
        <WebViewer rrd={PROXY} width="100%" height="520px" hide_welcome_screen />
      </div>
    </div>
  );
}
