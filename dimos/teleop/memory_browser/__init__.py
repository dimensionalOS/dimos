"""Memory Browser — Minority-Report-style VR UI for DimOS memory.

Subclasses :class:`QuestTeleopModule` to reuse its WebSocket + controller plumbing,
adds a reverse channel that pushes thumbnails from a memory store to the headset,
and a gesture layer (palm-roll scrub, swipe-dismiss) that drives a 3D timeline
ribbon rendered in the Quest browser via Three.js.
"""

from dimos.teleop.memory_browser.module import (
    MemoryBrowserConfig,
    MemoryBrowserModule,
)

__all__ = ["MemoryBrowserConfig", "MemoryBrowserModule"]
