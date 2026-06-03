from . import Surface

def circle(
    surface: Surface,
    color: tuple[int, int, int],
    center: tuple[int, int],
    radius: int,
    width: int = ...,
) -> None: ...
def rect(
    surface: Surface,
    color: tuple[int, int, int],
    rect: tuple[int, int, int, int],
    width: int = ...,
) -> None: ...
def line(
    surface: Surface,
    color: tuple[int, int, int],
    start_pos: tuple[int, int],
    end_pos: tuple[int, int],
    width: int = ...,
) -> None: ...
