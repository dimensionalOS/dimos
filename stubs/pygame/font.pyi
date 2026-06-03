from . import Surface

class Font:
    def __init__(self, filename: str | None, size: int) -> None: ...
    def render(
        self,
        text: str,
        antialias: bool,
        color: tuple[int, int, int],
        background: tuple[int, int, int] | None = ...,
    ) -> Surface: ...
