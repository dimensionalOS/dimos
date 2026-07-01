# dimoscope gateway — one FastAPI process that taps the robot bus (LCM + Zenoh) and serves the browser
# SDK over every transport: the /ws data plane, the /media camera plane, and the SSE / poll / WebRTC /
# WebTransport bench transports. Assembled by app.py::build_app; run it with `python -m gateway`.
# The shared in-process tap lives in bus.py — start there.
from .bus import Bus, Sample

__all__ = ["Bus", "Sample"]
