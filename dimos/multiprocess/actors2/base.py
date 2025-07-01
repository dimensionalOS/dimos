# Copyright 2025 Dimensional Inc.
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

"""Core building blocks for the *actors2* graph-stream framework.

Public surface
--------------
• In[T], Out[T] – local data streams
• RemoteIn[T], RemoteOut[T] – cross-process proxies
• Module – user subclass that represents a logical unit
• @module – decorator that wires IO descriptors and RPCs
• @rpc – tag to mark remotable methods
• State – simple lifecycle enum (for pretty printing only)
"""

from __future__ import annotations

import enum
import inspect
from typing import Any, Callable, Dict, Generic, List, TypeVar, get_args, get_origin, get_type_hints

from distributed.actor import Actor  # only imported for type-checking

from dimos.multiprocess.actors2 import colors
from dimos.multiprocess.actors2.o3dpickle import register_picklers

register_picklers()

T = TypeVar("T")


# ---------------------------------------------------------------------------
# Helper decorators
# ---------------------------------------------------------------------------


def rpc(fn: Callable[..., Any]) -> Callable[..., Any]:
    """Mark *fn* as remotely callable."""

    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn


# ---------------------------------------------------------------------------
# Stream primitives
# ---------------------------------------------------------------------------


class State(enum.Enum):
    DORMANT = "dormant"  # descriptor defined but not bound
    READY = "ready"  # bound to owner but not yet connected
    CONNECTED = "connected"  # input bound to an output
    FLOWING = "flowing"  # runtime: data observed


class Stream(Generic[T]):
    """Base class shared by *In* and *Out* streams."""

    def __init__(self, typ: type[T], name: str):
        self.type: type[T] = typ
        self.name: str = name

    # ------------------------------------------------------------------
    # Descriptor plumbing – auto-fill name when used as class attr
    # ------------------------------------------------------------------
    def __set_name__(self, owner: type, attr_name: str) -> None:  # noqa: D401
        if not getattr(self, "name", ""):
            self.name = attr_name

    # ------------------------------------------------------------------
    # String helpers ----------------------------------------------------
    # ------------------------------------------------------------------
    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def _color_fn(self) -> Callable[[str], str]:
        if self.state == State.DORMANT:
            return colors.orange
        if self.state == State.READY:
            return colors.blue
        if self.state == State.CONNECTED:
            return colors.green
        return lambda s: s

    def __str__(self) -> str:  # noqa: D401
        return self._color_fn()(f"{self.name}[{self.type_name}]")

    # ------------------------------------------------------------------
    # Lifecycle – subclasses implement .state
    # ------------------------------------------------------------------
    @property
    def state(self) -> State:  # pragma: no cover – abstract
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Outputs (producers)
# ---------------------------------------------------------------------------


class BaseOut(Stream[T]):
    """Common behaviour shared by *local* and *remote* outputs."""

    def __init__(self, typ: type[T], name: str = "Out", owner: Any | None = None):
        super().__init__(typ, name)
        self.owner: Any | None = owner

    @property
    def state(self) -> State:  # noqa: D401
        return State.DORMANT if self.owner is None else State.READY

    # API surface -------------------------------------------------------
    def publish(self, value: T) -> None:  # pragma: no cover – abstract
        raise NotImplementedError

    def subscribe(self, inp: "In[T]") -> None:  # pragma: no cover – abstract
        raise NotImplementedError


class Out(BaseOut[T]):
    """Local *Out* – synchronous fan-out to subscribers."""

    def __init__(self, typ: type[T], name: str = "Out", owner: Any | None = None):
        super().__init__(typ, name, owner)
        self._subscribers: List[In[T]] = []

    def publish(self, value: T) -> None:  # noqa: D401
        """Send *value* to all subscribers.

        • Local `In` → direct callback dispatch via ``_receive``
        • Remote `In` (its ``owner`` is a *distributed.Actor*) → perform a
          synchronous RPC so the receiving process can enqueue the message.
        """
        for inp in list(self._subscribers):
            owner = getattr(inp, "owner", None)

            if isinstance(owner, Actor):
                # Cross-process: schedule RPC on remote actor.
                try:
                    getattr(owner, "receive_msg")(inp.name, value)  # type: ignore[misc]
                except Exception:  # pylint: disable=broad-except
                    continue  # swallow network issues during shutdown
            else:
                # In-process delivery.
                inp._receive(value)

    def subscribe(self, inp: "In[T]") -> None:  # noqa: D401
        if inp not in self._subscribers:
            self._subscribers.append(inp)

    def __reduce__(self):  # noqa: D401
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (RemoteOut, (self.type, self.name, self.owner.ref))


class RemoteOut(BaseOut[T]):
    """Proxy for an *Out* that lives on a remote *distributed.Actor*."""

    def __init__(self, typ: type[T], name: str, owner: Actor | None = None):
        super().__init__(typ, name, owner)

    def subscribe(self, inp: "In[T]") -> None:  # noqa: D401
        if self.owner is None:
            raise RuntimeError("RemoteOut has no associated Actor; cannot subscribe")
        fut = self.owner.subscribe(self.name, inp)
        try:
            fut.result()
        except AttributeError:
            pass  # non-future – best effort


# ---------------------------------------------------------------------------
# Inputs (consumers)
# ---------------------------------------------------------------------------


class In(Stream[T]):
    """Local *In* – pull side of the data flow."""

    def __init__(
        self,
        typ: type[T],
        name: str = "In",
        owner: Any | None = None,
        source: BaseOut[T] | None = None,
    ) -> None:
        super().__init__(typ, name)
        self.owner: Any | None = owner
        self.source: BaseOut[T] | None = source
        self._callbacks: List[Callable[[T], None]] = []

    # ------------------------------------------------------------------
    # Introspection helpers
    # ------------------------------------------------------------------
    @property
    def state(self) -> State:  # noqa: D401
        return State.CONNECTED if self.source else State.DORMANT

    def __str__(self) -> str:  # noqa: D401
        if self.state == State.CONNECTED and self.source is not None:
            return f"IN {super().__str__()} <- {self.source}"
        return f"IN {super().__str__()}"

    # ------------------------------------------------------------------
    # Connectivity API
    # ------------------------------------------------------------------
    def bind(self, out_stream: BaseOut[T]) -> None:
        if self.source is not None:
            raise RuntimeError("Input already connected")
        self.source = out_stream
        out_stream.subscribe(self)

    # Backwards-compat alias
    connect = bind  # type: ignore[attr-defined]

    def subscribe(self, callback: Callable[[T], None]) -> None:  # noqa: D401
        if self.source is None:
            raise ValueError("Cannot subscribe to an unconnected In stream")
        if not self._callbacks:
            self.source.subscribe(self)
        self._callbacks.append(callback)

    # ------------------------------------------------------------------
    # Internal helper – called by Out.publish
    # ------------------------------------------------------------------
    def _receive(self, value: T) -> None:
        for cb in list(self._callbacks):
            cb(value)

    # ------------------------------------------------------------------
    # Pickling – becomes RemoteIn on the other side
    # ------------------------------------------------------------------
    def __reduce__(self):  # noqa: D401
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise In without an owner ref")
        return (RemoteIn, (self.type, self.name, self.owner.ref))


class RemoteIn(In[T]):
    """Proxy for an *In* that lives on a remote actor."""

    def __init__(self, typ: type[T], name: str, owner: Actor):
        super().__init__(typ, name, owner, None)

    def __str__(self) -> str:  # noqa: D401
        return f"{self.__class__.__name__} {super().__str__()} @ {self.owner}"


# ---------------------------------------------------------------------------
# Module infrastructure
# ---------------------------------------------------------------------------


class Module:  # pylint: disable=too-few-public-methods
    """Base-class for user logic blocks (actors)."""

    inputs: Dict[str, In[Any]] = {}
    outputs: Dict[str, Out[Any]] = {}
    rpcs: Dict[str, Callable[..., Any]] = {}

    # ------------------------------------------------------------------
    # Runtime helpers
    # ------------------------------------------------------------------
    def bind(self, input_name: str, source: Out[Any]) -> None:
        inp = In(source.type, input_name, self, source)
        self.inputs[input_name] = inp
        setattr(self, input_name, inp)

    connect = bind  # legacy alias

    def subscribe(self, output_name: str, remote_input: In[Any]) -> None:  # noqa: D401
        getattr(self, output_name).subscribe(remote_input)

    def receive_msg(self, input_name: str, msg: Any) -> None:  # noqa: D401
        self.inputs[input_name]._receive(msg)

    def set_ref(self, ref: Any) -> None:  # noqa: D401
        self.ref = ref  # created dynamically elsewhere

    def __str__(self) -> str:  # noqa: D401
        return f"{self.__class__.__name__}-Local"

    @classmethod
    def io(cls) -> str:  # noqa: D401
        def _boundary(seq, first: str, mid: str, last: str):
            seq = list(seq)
            for idx, s in enumerate(seq):
                if idx == 0:
                    yield first + s
                elif idx == len(seq) - 1:
                    yield last + s
                else:
                    yield mid + s

        def _box(name: str) -> str:
            return "\n".join(
                [
                    "┌┴" + "─" * (len(name) + 1) + "┐",
                    f"│ {name} │",
                    "└┬" + "─" * (len(name) + 1) + "┘",
                ]
            )

        inputs = list(_boundary(map(str, cls.inputs.values()), " ┌─ ", " ├─ ", " ├─ "))

        # RPC signatures -------------------------------------------------
        rpc_lines: List[str] = []
        for n, fn in cls.rpcs.items():
            sig = inspect.signature(fn)
            hints = get_type_hints(fn, include_extras=True)
            params: List[str] = []
            for p in sig.parameters:
                if p in ("self", "cls"):
                    continue
                ann = hints.get(p, Any)
                params.append(f"{p}: {getattr(ann, '__name__', repr(ann))}")
            ret_ann = hints.get("return", Any)
            rpc_lines.append(
                f"{n}({', '.join(params)}) → {getattr(ret_ann, '__name__', repr(ret_ann))}"
            )

        rpcs = list(_boundary(rpc_lines, " ├─ ", " ├─ ", " └─ "))

        outputs = list(
            _boundary(
                map(str, cls.outputs.values()),
                " ├─ ",
                " ├─ ",
                " ├─ " if rpcs else " └─ ",
            )
        )

        if rpcs:
            rpcs.insert(0, " │")

        return "\n".join(inputs + [_box(cls.__name__)] + outputs + rpcs)


# ---------------------------------------------------------------------------
# @module decorator – reflection heavy-lifting
# ---------------------------------------------------------------------------


def module(cls: type) -> type:  # noqa: D401
    """Decorate *cls* to inject IO descriptors and RPC metadata."""

    # Guarantee dicts are *per-class*, not shared between subclasses
    cls.inputs = dict(getattr(cls, "inputs", {}))  # type: ignore[attr-defined]
    cls.outputs = dict(getattr(cls, "outputs", {}))  # type: ignore[attr-defined]
    cls.rpcs = dict(getattr(cls, "rpcs", {}))  # type: ignore[attr-defined]

    # 1) Handle class-level annotations --------------------------------
    for name, ann in get_type_hints(cls, include_extras=True).items():
        origin = get_origin(ann)
        if origin is Out:
            inner, *_ = get_args(ann) or (Any,)
            stream = Out(inner, name)
            cls.outputs[name] = stream
            setattr(cls, name, stream)
        elif origin is In:
            inner, *_ = get_args(ann) or (Any,)
            stream = In(inner, name)
            cls.inputs[name] = stream
            setattr(cls, name, stream)

    # 2) Gather RPCs ----------------------------------------------------
    for n, obj in cls.__dict__.items():
        if callable(obj) and getattr(obj, "__rpc__", False):
            cls.rpcs[n] = obj

    # 3) Wrap __init__ --------------------------------------------------
    original_init = cls.__init__  # type: ignore[attr-defined]

    def _init_wrapper(self, *args, **kwargs):  # noqa: D401 – inner func
        # (a) bind owners for pre-declared streams
        for s in cls.outputs.values():
            s.owner = self
        for s in cls.inputs.values():
            s.owner = self

        # (b) convert RemoteOut kwargs → connected In
        new_kwargs = {}
        for k, v in kwargs.items():
            if isinstance(v, RemoteOut):
                inp = In(v.type, v.name, self, v)
                cls.inputs[k] = inp
                new_kwargs[k] = inp
            else:
                new_kwargs[k] = v

        # (c) delegate
        original_init(self, *args, **new_kwargs)

    cls.__init__ = _init_wrapper  # type: ignore[assignment]

    return cls


__all__ = [
    "In",
    "Out",
    "RemoteIn",
    "RemoteOut",
    "Module",
    "module",
    "rpc",
    "State",
]
