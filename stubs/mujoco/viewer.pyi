from contextlib import AbstractContextManager
from typing import Any

from . import MjData, MjModel

class _Camera:
    lookat: Any
    distance: float
    azimuth: float
    elevation: float

class Handle(AbstractContextManager["Handle"]):
    cam: _Camera
    def is_running(self) -> bool: ...
    def sync(self) -> None: ...
    def close(self) -> None: ...
    def __exit__(self, *args: object) -> None: ...

def launch_passive(
    model: MjModel,
    data: MjData,
    *,
    show_left_ui: bool = ...,
    show_right_ui: bool = ...,
    key_callback: Any = ...,
) -> Handle: ...
