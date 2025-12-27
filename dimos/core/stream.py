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

from __future__ import annotations

import enum
from typing import (
    TYPE_CHECKING,
    Any,
    Generic,
    TypeVar,
)

from dask.distributed import Actor
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable

import dimos.core.colors as colors
from dimos.core.resource import Resource
from dimos.utils.logging_config import setup_logger
import dimos.utils.reactive as reactive
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from collections.abc import Callable

    from reactivex.observable import Observable

T = TypeVar("T")


logger = setup_logger()


class ObservableMixin(Generic[T]):
    # subscribes and returns the first value it receives
    # might be nicer to write without rxpy but had this snippet ready
    def get_next(self, timeout: float = 10.0) -> T:
        try:
            return (  # type: ignore[no-any-return]
                self.observable()  # type: ignore[no-untyped-call]
                .pipe(ops.first(), *([ops.timeout(timeout)] if timeout is not None else []))
                .run()
            )
        except Exception as e:
            raise Exception(f"No value received after {timeout} seconds") from e

    def hot_latest(self) -> Callable[[], T]:
        return reactive.getter_streaming(self.observable())  # type: ignore[no-untyped-call]

    def pure_observable(self) -> Observable[T]:
        def _subscribe(observer, scheduler=None):  # type: ignore[no-untyped-def]
            unsubscribe = self.subscribe(observer.on_next)  # type: ignore[attr-defined]
            return Disposable(unsubscribe)

        return rx.create(_subscribe)

    # default return is backpressured because most
    # use cases will want this by default
    def observable(self):  # type: ignore[no-untyped-def]
        return backpressure(self.pure_observable())


class State(enum.Enum):
    UNBOUND = "unbound"  # descriptor defined but not bound
    READY = "ready"  # bound to owner but not yet connected
    CONNECTED = "connected"  # input bound to an output
    FLOWING = "flowing"  # runtime: data observed


class Transport(Resource, ObservableMixin[T]):
    # used by local Output
    def broadcast(self, selfstream: Out[T], value: T) -> None: ...

    def publish(self, msg: T) -> None:
        self.broadcast(None, msg)  # type: ignore[arg-type]

    # used by local Input
    def subscribe(self, selfstream: In[T], callback: Callable[[T], any]) -> None: ...  # type: ignore[valid-type]


class Stream(Generic[T]):
    _transport: Transport | None  # type: ignore[type-arg]

    def __init__(
        self,
        type: type[T],
        name: str,
        owner: Any | None = None,
        transport: Transport | None = None,  # type: ignore[type-arg]
    ) -> None:
        self.name = name
        self.owner = owner
        self.type = type
        if transport:
            self._transport = transport
        if not hasattr(self, "_transport"):
            self._transport = None

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def _color_fn(self) -> Callable[[str], str]:
        if self.state == State.UNBOUND:  # type: ignore[attr-defined]
            return colors.orange
        if self.state == State.READY:  # type: ignore[attr-defined]
            return colors.blue
        if self.state == State.CONNECTED:  # type: ignore[attr-defined]
            return colors.green
        return lambda s: s

    def __str__(self) -> str:
        return (
            self.__class__.__name__
            + " "
            + self._color_fn()(f"{self.name}[{self.type_name}]")
            + " @ "
            + (
                colors.orange(self.owner)  # type: ignore[arg-type]
                if isinstance(self.owner, Actor)
                else colors.green(self.owner)  # type: ignore[arg-type]
            )
            + ("" if not self._transport else " via " + str(self._transport))
        )


class Out(Stream[T]):
    _transport: Transport  # type: ignore[type-arg]

    def __init__(self, *argv, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*argv, **kwargs)

    @property
    def transport(self) -> Transport[T]:
        return self._transport

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        # just for type checking
        ...

    @property
    def state(self) -> State:
        return State.UNBOUND if self.owner is None else State.READY

    def __reduce__(self):  # type: ignore[no-untyped-def]
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (
            RemoteOut,
            (
                self.type,
                self.name,
                self.owner.ref,
                self._transport,
            ),
        )

    def publish(self, msg) -> None:  # type: ignore[no-untyped-def]
        if not hasattr(self, "_transport") or self._transport is None:
            logger.warning(f"Trying to publish on Out {self} without a transport")
            return
        self._transport.broadcast(self, msg)


class RemoteStream(Stream[T]):
    @property
    def state(self) -> State:
        return State.UNBOUND if self.owner is None else State.READY

    # this won't work but nvm
    @property
    def transport(self) -> Transport[T]:
        return self._transport  # type: ignore[return-value]

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        self.owner.set_transport(self.name, value).result()  # type: ignore[union-attr]
        self._transport = value


class RemoteOut(RemoteStream[T]):
    def connect(self, other: RemoteIn[T]):  # type: ignore[no-untyped-def]
        return other.connect(self)

    def subscribe(self, cb) -> Callable[[], None]:  # type: ignore[no-untyped-def]
        return self.transport.subscribe(cb, self)  # type: ignore[arg-type, func-returns-value, no-any-return]


# representation of Input
# as views from inside of the module
class In(Stream[T], ObservableMixin[T]):
    connection: RemoteOut[T] | None = None
    _transport: Transport  # type: ignore[type-arg]

    def __str__(self) -> str:
        mystr = super().__str__()

        if not self.connection:
            return mystr

        return (mystr + " ◀─").ljust(60, "─") + f" {self.connection}"

    def __reduce__(self):  # type: ignore[no-untyped-def]
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (RemoteIn, (self.type, self.name, self.owner.ref, self._transport))

    @property
    def transport(self) -> Transport[T]:
        if not self._transport and self.connection:
            self._transport = self.connection.transport  # type: ignore[union-attr]
        return self._transport

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        # just for type checking
        ...

    def connect(self, value: Out[T]) -> None:
        # just for type checking
        ...

    @property
    def state(self) -> State:
        return State.UNBOUND if self.owner is None else State.READY

    # returns unsubscribe function
    def subscribe(self, cb) -> Callable[[], None]:  # type: ignore[no-untyped-def]
        return self.transport.subscribe(cb, self)  # type: ignore[arg-type, func-returns-value, no-any-return]


# representation of input outside of module
# used for configuring connections, setting a transport
class RemoteIn(RemoteStream[T]):
    def connect(self, other: RemoteOut[T]) -> None:
        return self.owner.connect_stream(self.name, other).result()  # type: ignore[no-any-return, union-attr]

    # this won't work but that's ok
    @property  # type: ignore[misc]
    def transport(self) -> Transport[T]:
        return self._transport  # type: ignore[return-value]

    def publish(self, msg) -> None:  # type: ignore[no-untyped-def]
        self.transport.broadcast(self, msg)  # type: ignore[arg-type]

    @transport.setter  # type: ignore[attr-defined, misc, no-redef, untyped-decorator]
    def transport(self, value: Transport[T]) -> None:
        self.owner.set_transport(self.name, value).result()  # type: ignore[union-attr]
        self._transport = value
