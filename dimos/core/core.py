#!/usr/bin/env python3
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
import inspect
import traceback
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

from dask.distributed import Actor

import dimos.core.colors as colors
from dimos.core.o3dpickle import register_picklers

register_picklers()
T = TypeVar("T")


class Transport(Protocol[T]):
    # used by local Output
    def broadcast(self, selfstream: Out[T], value: T): ...

    # used by local Input
    def subscribe(self, selfstream: In[T], callback: Callable[[T], any]) -> None: ...


class DaskTransport(Transport[T]):
    subscribers: List[Callable[[T], None]]
    _started: bool = False

    def __init__(self):
        self.subscribers = []

    def __str__(self) -> str:
        return colors.yellow("DaskTransport")

    def __reduce__(self):
        return (DaskTransport, ())

    def broadcast(self, selfstream: RemoteIn[T], msg: T) -> None:
        for subscriber in self.subscribers:
            # there is some sort of a bug here with losing worker loop
            #            print(subscriber.owner, subscriber.owner._worker, subscriber.owner._client)
            #            subscriber.owner._try_bind_worker_client()
            #            print(subscriber.owner, subscriber.owner._worker, subscriber.owner._client)

            subscriber.owner.dask_receive_msg(subscriber.name, msg).result()

    def dask_receive_msg(self, msg) -> None:
        for subscriber in self.subscribers:
            try:
                subscriber(msg)
            except Exception as e:
                print(
                    colors.red("Error in DaskTransport subscriber callback:"),
                    e,
                    traceback.format_exc(),
                )

    # for outputs
    def dask_register_subscriber(self, remoteInput: RemoteIn[T]) -> None:
        self.subscribers.append(remoteInput)

    # for inputs
    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            selfstream.connection.owner.dask_register_subscriber(
                selfstream.connection.name, selfstream
            ).result()
            self._started = True
        self.subscribers.append(callback)


class State(enum.Enum):
    UNBOUND = "unbound"  # descriptor defined but not bound
    READY = "ready"  # bound to owner but not yet connected
    CONNECTED = "connected"  # input bound to an output
    FLOWING = "flowing"  # runtime: data observed


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
        if not hasattr(self, "_transport") or self._transport is None:
            self._transport = DaskTransport()

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


class In(Stream[T]):
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

    def subscribe(self, cb):
        self.transport.subscribe(self, cb)


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


def rpc(fn: Callable[..., Any]) -> Callable[..., Any]:
    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn


daskTransport = DaskTransport()  # singleton instance for use in Out/RemoteOut
