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

from dimos.multiprocess.actors2 import colors
from dimos.multiprocess.actors2.o3dpickle import register_picklers

register_picklers()
T = TypeVar("T")


class Transport(Protocol[T]):
    # used by local Output
    def broadcast(self, selfstream: Out[T], value: T): ...

    # used by remote Input
    def connect(self, selfstream: RemoteIn[T], otherstream: RemoteOut[T]) -> None: ...

    # used by local Input
    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None: ...


class DaskTransport(Transport[T]):
    def __str__(self) -> str:
        return "DaskTransport"

    # used by remote Input
    def connect(self, selfstream: RemoteIn[T], otherstream: RemoteOut[T]) -> None:
        print("dask transport connection request")
        print(selfstream, "->", otherstream)

    def broadcast(self, selfstream: Out[T], value: T): ...


class LCMTransport(Transport[T]):
    def broadcast(self, selfstream: Out[T], value: T): ...


class State(enum.Enum):
    UNBOUND = "unbound"  # descriptor defined but not bound
    READY = "ready"  # bound to owner but not yet connected
    CONNECTED = "connected"  # input bound to an output
    FLOWING = "flowing"  # runtime: data observed


class Stream(Generic[T]):
    _transport = DaskTransport()

    def __init__(self, type: type[T], name: str, owner: Any | None = None):
        self.name = name
        self.owner = owner
        self.type = type

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
            + ("" if not self._transport else " - " + colors.yellow(str(self._transport)))
        )


class Out(Stream[T]):
    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY

    def __reduce__(self):  # noqa: D401
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (RemoteOut, (self.type, self.name, self.owner.ref))

    def publish(self, msg): ...


class RemoteStream(Stream[T]):
    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY

    @property
    def transport(self) -> Transport[T]:
        return self._transport

    @transport.setter
    def transport(self, value: Transport[T]) -> None:
        self.owner.set_transport(self.name, value).result()


class RemoteOut(RemoteStream[T]):
    def connect(self, other: RemoteIn[T]):
        print("sub request from", self, "to", other)


class In(Stream[T]):
    connection: Optional[RemoteOut[T]] = None

    def __str__(self):
        return super().__str__() + ("" if not self.connection else f" <- {self.connection}")

    def subscribe(self, cb): ...

    def __reduce__(self):  # noqa: D401
        if self.owner is None or not hasattr(self.owner, "ref"):
            raise ValueError("Cannot serialise Out without an owner ref")
        return (RemoteIn, (self.type, self.name, self.owner.ref))

    @property
    def state(self) -> State:  # noqa: D401
        return State.UNBOUND if self.owner is None else State.READY


class RemoteIn(RemoteStream[T]):
    def connect(self, other: RemoteOut[T]) -> None:
        return self.owner.connect_stream(self.name, other).result()


def rpc(fn: Callable[..., Any]) -> Callable[..., Any]:
    """Mark *fn* as remotely callable."""

    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn


daskTransport = DaskTransport()  # singleton instance for use in Out/RemoteOut


# process for LCM
# remoteInput - connect to remoteOutput
# or remoteOutput - connect to remoteInput
#
# remoteInput learns the actual LCM topic, from that point on local comms


# process for Dask
# remoteInput - connect to remoteOutput
# or remoteOutput - connect to remoteInput
#
# remoteInput learns the actual Dask actor from remoteOutput
# remoteInput contacts the actor, telling it "I'm interested in remoteOutput, contact me here"


# this means that transport split is at
# remoteInput/remoteOutput sub request level?
#
# remoteInput needs to communicate the transport to local Input in some way, so Transport def is portable.
