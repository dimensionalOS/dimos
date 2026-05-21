class Event:
    type: int
    key: int
    mod: int
    pos: tuple[int, int]
    button: int

def get() -> list[Event]: ...
def pump() -> None: ...
def post(event: Event) -> bool: ...
