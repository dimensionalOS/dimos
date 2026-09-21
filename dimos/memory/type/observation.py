# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from dataclasses import dataclass, fields
import sys
import threading
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.models.embedding.base import Embedding

T = TypeVar("T")
R = TypeVar("R")

PoseTuple = tuple[float, float, float, float, float, float, float]


class _Unloaded:
    """Sentinel indicating data has not been loaded yet."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "<unloaded>"


_UNLOADED = _Unloaded()


class _MissingType:
    """Sentinel for "argument not supplied", distinct from None."""

    def __repr__(self) -> str:
        return "<missing>"


_MISSING = _MissingType()


def _to_tuple(p: Any) -> PoseTuple | None:
    """Coerce common pose shapes to the storage 7-tuple `(x, y, z, qx, qy, qz, qw)`.

    Accepts: `None`, a 3- or 7-tuple, or any object with
    `translation`+`rotation` (Transform) or `position`+`orientation`
    (Pose / PoseStamped) attributes. 3-tuples are padded with the identity
    quaternion. Duck-typed to avoid importing `dimos.msgs.geometry_msgs`
    at module load.
    """
    if p is None:
        return None
    if isinstance(p, (tuple, list)):
        if len(p) == 7:
            return cast("PoseTuple", tuple(float(v) for v in p))
        if len(p) == 3:
            x, y, z = p
            return (float(x), float(y), float(z), 0.0, 0.0, 0.0, 1.0)
        raise TypeError(f"Pose tuple must have length 3 or 7, got {len(p)}")
    # Use ``is not None`` rather than ``or`` — a Vector3 at the origin is
    # falsy but valid, and falling through to ``position`` on a Transform
    # would crash.
    trans = getattr(p, "translation", None)
    if trans is None:
        trans = getattr(p, "position", None)
    rot = getattr(p, "rotation", None)
    if rot is None:
        rot = getattr(p, "orientation", None)
    if trans is None or rot is None:
        raise TypeError(
            f"Cannot coerce {type(p).__name__} to a pose tuple — expected "
            "tuple, None, Transform, Pose, or PoseStamped."
        )
    return (
        float(trans.x),
        float(trans.y),
        float(trans.z),
        float(rot.x),
        float(rot.y),
        float(rot.z),
        float(rot.w),
    )


@dataclass(init=False)
class Observation(Generic[T]):
    """A single timestamped observation with optional spatial pose and metadata.

    Pose is stored internally as a 7-tuple ``(x, y, z, qx, qy, qz, qw)``.
    Use :attr:`pose` for a typed :class:`Pose` object (lazy), or
    :attr:`pose_tuple` for direct field access in hot loops.
    """

    id: int
    ts: float
    data_type: type
    pose_tuple: PoseTuple | None
    tags: dict[str, Any]
    _data: T | _Unloaded
    _loader: Callable[[], T] | None
    _data_lock: threading.Lock

    def __init__(
        self,
        id: int = 0,
        ts: float = 0.0,
        *,
        data_type: type = object,
        pose: Any | _MissingType = _MISSING,
        pose_tuple: PoseTuple | None = None,
        tags: dict[str, Any] | None = None,
        _data: T | _Unloaded = _UNLOADED,
        _loader: Callable[[], T] | None = None,
        _data_lock: threading.Lock | None = None,
    ) -> None:
        self.id = id
        self.ts = ts
        self.data_type = data_type

        # `pose` wins if explicitly supplied (including `pose=None`, which
        # clears). `derive()`/`tag()` re-pass the current `pose_tuple` from
        # fields(); only an explicit override on `pose` should beat it.
        if pose is _MISSING:
            self.pose_tuple = pose_tuple
        else:
            self.pose_tuple = _to_tuple(pose)
        self.tags = tags if tags is not None else {}
        self._data = _data
        self._loader = _loader
        self._data_lock = _data_lock if _data_lock is not None else threading.Lock()

    @property
    def pose(self) -> Pose | None:
        """Typed :class:`Pose` (or None). Allocates per access — read :attr:`pose_tuple` in hot loops."""
        if self.pose_tuple is None:
            return None

        return Pose(*self.pose_tuple)

    @property
    def pose_stamped(self) -> PoseStamped | None:
        """Typed :class:`PoseStamped` (or None) carrying this observation's ts."""
        if self.pose_tuple is None:
            return None
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        x, y, z, qx, qy, qz, qw = self.pose_tuple
        return PoseStamped(ts=self.ts, position=(x, y, z), orientation=(qx, qy, qz, qw))

    @property
    def data(self) -> T:
        val = self._data
        if isinstance(val, _Unloaded):
            with self._data_lock:
                # Re-check after acquiring lock (double-checked locking)
                val = self._data
                if isinstance(val, _Unloaded):
                    if self._loader is None:
                        raise LookupError("No data and no loader set on this observation")
                    loaded = self._loader()
                    self._data = loaded
                    self._loader = None  # release closure
                    return loaded
            return val
        return val

    def derive(self, *, data: R, **overrides: Any) -> Observation[R]:
        """New observation with replaced ``data``; other fields carry over.

        Passing ``embedding`` on a plain :class:`Observation` promotes it to
        :class:`EmbeddedObservation`.
        """
        cls: type[Observation[Any]] = (
            EmbeddedObservation
            if "embedding" in overrides and not isinstance(self, EmbeddedObservation)
            else type(self)
        )
        kwargs: dict[str, Any] = {f.name: getattr(self, f.name) for f in fields(self)}
        kwargs.update(overrides)
        kwargs.update(data_type=type(data), _data=data, _loader=None)
        return cast("Observation[R]", cls(**kwargs))

    def tag(self, **tags: Any) -> Self:
        """Return a new observation with tags merged in."""
        kwargs: dict[str, Any] = {f.name: getattr(self, f.name) for f in fields(self)}
        kwargs.update(
            tags={**self.tags, **tags},
            _data=_UNLOADED,
            _loader=lambda: self.data,
            _data_lock=threading.Lock(),
        )
        return type(self)(**kwargs)

    def with_pose(self, pose: Any) -> Self:
        """Return a new observation with ``pose`` attached, payload kept lazy.

        ``pose`` accepts anything :func:`_to_tuple` handles (a 3-/7-tuple,
        ``Pose``/``PoseStamped``/``Transform``, or ``None`` to clear).
        """
        kwargs: dict[str, Any] = {f.name: getattr(self, f.name) for f in fields(self)}
        kwargs.pop("pose_tuple", None)
        kwargs.update(
            pose=pose,
            _data=_UNLOADED,
            _loader=lambda: self.data,
            _data_lock=threading.Lock(),
        )
        return type(self)(**kwargs)


@dataclass(init=False)
class EmbeddedObservation(Observation[T]):
    """Observation enriched with a vector embedding and optional similarity score."""

    embedding: Embedding | None
    similarity: float | None

    def __init__(
        self,
        id: int = 0,
        ts: float = 0.0,
        *,
        data_type: type = object,
        pose: Any | _MissingType = _MISSING,
        pose_tuple: PoseTuple | None = None,
        tags: dict[str, Any] | None = None,
        embedding: Embedding | None = None,
        similarity: float | None = None,
        _data: T | _Unloaded = _UNLOADED,
        _loader: Callable[[], T] | None = None,
        _data_lock: threading.Lock | None = None,
    ) -> None:
        super().__init__(
            id=id,
            ts=ts,
            data_type=data_type,
            pose=pose,
            pose_tuple=pose_tuple,
            tags=tags,
            _data=_data,
            _loader=_loader,
            _data_lock=_data_lock,
        )
        self.embedding = embedding
        self.similarity = similarity
