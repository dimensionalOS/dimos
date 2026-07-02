# The bench delivery transports: SSE and HTTP long-poll tap the shared Bus and re-emit the same
# [f64 send-ms][LC02] frames as the default WS data plane, so the browser decodes every path
# identically. They exist to benchmark delivery mechanisms against the WebSocket baseline (e.g. TCP
# head-of-line blocking under loss); both are read-only and mount on their own paths. WebTransport
# AND WebRTC live in the native sidecar (gateway/wt-sidecar), fed by gateway/pipe.py — /rtc here is
# only the SDP signaling relay (webrtc.py). _common.py holds the framing helpers.
from .poll import PollPlane
from .sse import SsePlane
from .webrtc import RtcSignalRelay

__all__ = ["PollPlane", "RtcSignalRelay", "SsePlane"]
