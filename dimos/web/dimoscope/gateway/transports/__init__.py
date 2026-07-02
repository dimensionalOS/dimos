# The bench delivery transports: SSE, HTTP long-poll, WebRTC DataChannel. Each taps the shared Bus and
# re-emits the same [f64 send-ms][LC02] frames as the default WS data plane, so the browser decodes every
# path identically. They exist to benchmark delivery mechanisms against the WebSocket baseline (e.g. TCP
# head-of-line blocking under loss). All are read-only (control stays on the data WS); each mounts on its
# own path and all run at once. WebTransport lives in the native sidecar (gateway/wt-sidecar), fed by
# gateway/pipe.py; _common.py holds the framing helpers both share.
from .poll import PollPlane
from .sse import SsePlane
from .webrtc import WebRtcDataPlane

__all__ = ["PollPlane", "SsePlane", "WebRtcDataPlane"]
