import threading
import time
from dataclasses import dataclass, field
from abc import ABC
from typing import Tuple, Callable, Optional
from dimos.types.path import Path
from dimos.types.vector import Vector

import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject
from dimos.web.websocket_vis.types import Drawable


class Visualizable(ABC):
    """
    Base class for objects that can provide visualization data.
    """

    def vis_stream(self) -> Observable[Tuple[str, Drawable]]:
        if not hasattr(self, "_vis_subject"):
            self._vis_subject = Subject()
        return self._vis_subject

    def vis(self, name: str, drawable: Drawable) -> None:
        if not hasattr(self, "_vis_subject"):
            return
        self._vis_subject.on_next((name, drawable))


def vector_stream(
    name: str, pos: Callable[[], Vector], update_interval=0.1, precision=0.25, history=10
) -> Observable[Tuple[str, Drawable]]:
    return rx.interval(update_interval).pipe(
        ops.map(lambda _: pos()),
        ops.distinct_until_changed(
            comparer=lambda a, b: (a - b).length() < precision,
        ),
        ops.scan(
            lambda hist, cur: hist.ipush(cur).iclip_tail(history),
            seed=Path(),
        ),
        ops.flat_map(lambda path: rx.from_([(f"{name}_hst", path), (name, path.last())])),
    )
