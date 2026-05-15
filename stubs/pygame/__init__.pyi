"""pygame is ~30 MB, so we have a stub to avoid installing in lint job."""

from typing import Any

# --- Surface / Rect ---------------------------------------------------

class Surface:
    def fill(self, color: tuple[int, int, int]) -> None: ...
    def blit(
        self,
        source: Surface,
        dest: tuple[int, int] | tuple[float, float],
        area: Any | None = ...,
    ) -> None: ...
    def get_size(self) -> tuple[int, int]: ...

# --- top-level ---------------------------------------------------------

def init() -> tuple[int, int]: ...
def quit() -> None: ...

# Constants

SWSURFACE: int
QUIT: int
KEYDOWN: int
KEYUP: int

# Key constants — populate the ones dimos imports.
K_0: int
K_1: int
K_2: int
K_3: int
K_4: int
K_5: int
K_6: int
K_7: int
K_8: int
K_9: int
K_a: int
K_d: int
K_e: int
K_f: int
K_g: int
K_h: int
K_i: int
K_j: int
K_k: int
K_l: int
K_q: int
K_r: int
K_s: int
K_t: int
K_w: int
K_y: int
K_SPACE: int
K_ESCAPE: int
K_LCTRL: int
K_LSHIFT: int
K_RCTRL: int
K_RSHIFT: int

# --- submodules --------------------------------------------------------

from . import (
    display as display,
    draw as draw,
    event as event,
    font as font,
    key as key,
    time as time,
)
