# dimoscope gateway: one FastAPI process that taps the robot bus (LCM + Zenoh) and serves the browser SDK
# over every TCP transport (/ws data plane, /media camera plane, SSE/poll/WebRTC bench); WebTransport is
# the native sidecar (gateway/wt-sidecar), fed over a unix socket (pipe.py).
# Assembled by app.py::build_app; run with `python -m gateway`. The shared tap lives in bus.py.
from .bus import Bus, Sample

__all__ = ["Bus", "Sample"]
