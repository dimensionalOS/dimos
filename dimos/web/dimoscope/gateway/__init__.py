# dimoscope gateway: one FastAPI process that taps the robot bus (LCM + Zenoh) and serves the browser SDK
# over every transport (/ws data plane, /media camera plane, SSE/poll/WebRTC/WebTransport bench).
# Assembled by app.py::build_app; run with `python -m gateway`. The shared tap lives in bus.py.
from .bus import Bus, Sample

__all__ = ["Bus", "Sample"]
