from contextlib import AbstractContextManager
from typing import Any

from mujoco import MjData, MjModel, MjvOption, MjvPerturb

class _Camera:
    type: int
    trackbodyid: int
    lookat: Any
    distance: float
    azimuth: float
    elevation: float

class Handle(AbstractContextManager["Handle"]):
    cam: _Camera
    opt: MjvOption
    perturb: MjvPerturb
    def lock(self) -> AbstractContextManager[Any]: ...
    def is_running(self) -> bool: ...
    def sync(self, state_only: bool = ...) -> None: ...
    def set_texts(
        self,
        texts: tuple[int | None, int | None, str | None, str | None]
        | list[tuple[int | None, int | None, str | None, str | None]],
    ) -> None: ...
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
