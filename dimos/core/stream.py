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
    Any,
    Callable,
    Generic,
    Optional,
    TypeVar,
)

import reactivex as rx
from dask.distributed import Actor
from reactivex import operators as ops
from reactivex.disposable import Disposable

import dimos.core.colors as colors
import dimos.utils.reactive as reactive
from dimos.utils.reactive import backpressure

T = TypeVar("T")


class ObservableMixin(Generic[T]):
    # subscribes and returns the first value it receives
    # might be nicer to write without rxpy but had this snippet ready
    def get_next(self, timeout=10.0) -> T:
        try:
            return (
                self.observable()
                .pipe(ops.first(), *([ops.timeout(timeout)] if timeout is not None else []))
                .run()
            )
        except Exception as e:
            raise Exception(f"No value received after {timeout} seconds") from e

    def hot_latest(self) -> Callable[[], T]:
        return reactive.getter_streaming(self.observable())

    def pure_observable(self):
        def _subscribe(observer, scheduler=None):
            unsubscribe = self.subscribe(observer.on_next)
            return Disposable(unsubscribe)

        return rx.create(_subscribe)

    # default return is backpressured because most
    # use cases will want this by default
    def observable(self):
        return backpressure(self.pure_observable())


class State(enum.Enum):
    UNBOUND = "unbound"  # descriptor defined but not bound
    READY = "ready"  # bound to owner but not yet connected
    CONNECTED = "connected"  # input bound to an output
    FLOWING = "flowing"  # runtime: data observed


class Transport(ObservableMixin[T]):
    # used by local Output
    def broadcast(self, selfstream: Out[T], value: T): ...

    def publish(self, msg: T):
        self.broadcast(None, msg)

    # used by local Input
    def subscribe(self, selfstream: In[T], callback: Callable[[T], any]) -> None: ...


class Stream(Generic[T]):
    _transport: Optional[Transport]

    def __init__(
        self,
        type: type[T],
        name: str,
        owner: Optional[Any] = None,
        transport: Optional[Transport] = None,
    ):
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
        if self.state == State.UNBOUND:
            return colors.orange
        if self.state == State.READY:
            return colors.blue
        if self.state == State.CONNECTED:
            return colors.green
        return lambda s: s

    def __str__(self) -> str:  # noqa: D401
        return (
            self.__class__.__name__
            + " "
            + self._color_fn()(f"{self.name}[{self.type_name}]")
            + " @ "
            + (
                colors.orange(self.owner)
                if isinstance(self.owner, Actor)
                else colors.green(self.owner)
            )
            + ("" if not self._transport else " via " + str(self._transport))
        )


class Out(Stream[T]):
    _transport: Transport

    def __init__(self, *argv, **kwargs):
        super().__init__(*argv, **kwargs)

    @property
    def transport(self) -> Transport[T]:
        return self._transport

    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY

    def __reduce__(self):  # noqa: D401
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

    def publish(self, msg):
        if not hasattr(self, "_transport") or self._transport is None:
            raise Exception(f"{self} transport for stream is not specified,")
        self._transport.broadcast(self, msg)


class RemoteStream(Stream[T]):
    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY

    # this won't work but nvm
    @property
    def transport(self) -> Transport[T]:
        return self._transport

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        self.owner.set_transport(self.name, value).result()
        self._transport = value


class RemoteOut(RemoteStream[T]):
    def connect(self, other: RemoteIn[T]):
        return other.connect(self)

    def subscribe(self, cb) -> Callable[[], None]:
        return self.transport.subscribe(cb, self)


# representation of Input
# as views from inside of the module
class In(Stream[T], ObservableMixin[T]):
    connection: Optional[RemoteOut[T]] = None
    _transport: Transport

    def __str__(self):
        mystr = super().__str__()

        if not self.connection:
            return mystr

        return (mystr + " ◀─").ljust(60, "─") + f" {self.connection}"

    def __reduce__(self):  # noqa: D401
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (RemoteIn, (self.type, self.name, self.owner.ref, self._transport))

    @property
    def transport(self) -> Transport[T]:
        if not self._transport:
            self._transport = self.connection.transport
        return self._transport

    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY

    # returns unsubscribe function
    def subscribe(self, cb) -> Callable[[], None]:
        return self.transport.subscribe(cb, self)


# representation of input outside of module
# used for configuring connections, setting a transport
class RemoteIn(RemoteStream[T]):
    def connect(self, other: RemoteOut[T]) -> None:
        return self.owner.connect_stream(self.name, other).result()

    # this won't work but that's ok
    @property
    def transport(self) -> Transport[T]:
        return self._transport

    def publish(self, msg):
        self.transport.broadcast(self, msg)

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        self.owner.set_transport(self.name, value).result()
        self._transport = value
