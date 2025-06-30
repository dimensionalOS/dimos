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

import inspect
import time
from dataclasses import dataclass
from typing import Any, Generic, Optional, TypeVar, get_args, get_origin, get_type_hints

from dask.distributed import get_worker
from distributed.actor import Actor
from distributed.worker import thread_state

T = TypeVar("T")


def green(text: str) -> str:
    """Return the given text in green color."""
    return f"\033[92m{text}\033[0m"


def blue(text: str) -> str:
    """Return the given text in blue color."""
    return f"\033[94m{text}\033[0m"


def red(text: str) -> str:
    """Return the given text in red color."""
    return f"\033[91m{text}\033[0m"


def yellow(text: str) -> str:
    """Return the given text in yellow color."""
    return f"\033[93m{text}\033[0m"


def cyan(text: str) -> str:
    """Return the given text in cyan color."""
    return f"\033[96m{text}\033[0m"


def orange(text: str) -> str:
    """Return the given text in orange color."""
    return f"\033[38;5;208m{text}\033[0m"


class StreamDef(Generic[T]):
    def __init__(self, type: type[T], direction: str = "in"):
        self.type = type
        self.direction = direction  # 'in' or 'out'
        self.name: str | None = None

    def __set_name__(self, owner, n):
        self.name = n

    def __get__(self, *_):
        raise AttributeError("metadata only")

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))


def rpc(fn):
    fn.__rpc__ = True
    return fn


class In(Generic[T]):
    def __init__(self, type: type[T], name: str = "In"):
        self.type = type
        self.name = name

    def __set_name__(self, owner, n):
        self.name = n

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def __str__(self):
        return f"{self.name}[{self.type_name}]"

    def receive(self, message):
        """Receive a message on this input stream."""
        # For now, just pass - this can be extended later for processing
        print((time.perf_counter() - message.pubtime) * 1000)
        pass


@dataclass
class ActorReference:
    workerid: str
    actorid: str
    cls: type
    _actor: Optional[Actor] = None  # Store the actual deployed actor

    def __str__(self):
        return f"{(blue(self.actorid))}{green(self.workerid)}"

    @property
    def actor(self):
        # Return the stored actor if available, otherwise create a new one
        if self._actor is not None:
            return self._actor
        # Fallback to manual creation (this may not work properly for remote calls)
        return Actor(cls=self.cls, address=self.workerid, key=self.actorid)


# pattern 2 ── query the live WorkerState objects directly
def wid_to_addr(target, *, dask_scheduler=None):
    for a, ws in dask_scheduler.workers.items():
        if ws.server_id == target:  # exact match
            return a
    return None  # not found


class Out(Generic[T]):
    owner: Optional[ActorReference] = None
    context: Optional[ActorReference] = None
    inputkey: Optional[str] = None
    subscribers: list[tuple[ActorReference, str]] = []

    def __init__(self, type: type[T], name: str = "Out", owner: Any = None):
        self.type = type
        self.name = name
        if owner:
            # If owner is an Actor object, store it in the reference
            ref = owner.ref()
            if hasattr(owner, "__class__") and hasattr(owner, "_io_loop"):
                # This looks like a deployed Actor object
                ref._actor = owner
            self.owner = ref

    def __set_name__(self, owner, n):
        self.name = n

    # pickle control
    def __getstate__(self):
        state = self.__dict__.copy()
        state["subscribers"] = None
        return state

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def publish(self, value: T):
        if self.context:
            raise ValueError("You cannot publish to a remote actor stream")

        for sub in self.subscribers:
            (actor_ref, in_name) = sub
            print("PUB", value, "\nto", actor_ref, "input", in_name)
            try:
                actor_ref.actor.receive_message(in_name, value).result()
            except Exception as e:
                print(f"Error publishing to {actor_ref}: {e}")
                raise e

    def __str__(self):
        selfstr = orange(f"{self.name}[{self.type_name}]")
        if self.owner:
            if self.context:
                return f"{selfstr} from {self.owner} at {self.context}"
            return f"{selfstr} at {self.owner}"

        else:
            return selfstr

    def receive(self): ...

    def subscribe(self):
        if not self.context:
            raise ValueError(
                "Stream not within an Actor. Only actors can subscribe to Actors. keep the main loop free"
            )

        return self.owner.actor.subscribe(self.name, self.context, self.inputkey).result()


# ── decorator with *type-based* input / output detection ────────────────────
def module(cls: type) -> type:
    cls.inputs = dict(getattr(cls, "inputs", {}))
    cls.outputs = dict(getattr(cls, "outputs", {}))
    cls.rpcs = dict(getattr(cls, "rpcs", {}))

    cls_type_hints = get_type_hints(cls, include_extras=True)

    for n, ann in cls_type_hints.items():
        origin = get_origin(ann)
        # print(n, ann, origin)
        if origin is Out:
            inner_type, *_ = get_args(ann) or (Any,)
            md = Out(inner_type, n)
            cls.outputs[n] = md
            # make attribute accessible via instance / class
            setattr(cls, n, md)

    # RPCs
    for n, a in cls.__dict__.items():
        if callable(a) and getattr(a, "__rpc__", False):
            cls.rpcs[n] = a

    sig = inspect.signature(cls.__init__)
    type_hints = get_type_hints(cls.__init__, include_extras=True)

    # print(sig.parameters)
    for name, _ in sig.parameters.items():
        if name == "self":
            continue

        md = None
        ann = type_hints.get(name)
        origin = get_origin(ann)

        if origin is In:
            inner_type, *_ = get_args(ann) or (Any,)
            md = In(inner_type, name)

        if md is not None:
            cls.inputs[name] = md

    def _io_inner(c):
        def boundary_iter(iterable, first, middle, last):
            l = list(iterable)
            for idx, sd in enumerate(l):
                if idx == len(l) - 1:
                    yield last + sd
                elif idx == 0:
                    yield first + sd
                else:
                    yield middle + sd

        def box(name):
            top = "┌┴" + "─" * (len(name) + 1) + "┐"
            middle = f"│ {name} │"
            bottom = "└┬" + "─" * (len(name) + 1) + "┘"
            return f"{top}\n{middle}\n{bottom}"

        inputs = list(boundary_iter(map(str, c.inputs.values()), " ┌─ ", " ├─ ", " ├─ "))

        rpcs = []
        for n, fn in c.rpcs.items():
            sig = inspect.signature(fn)
            hints = get_type_hints(fn, include_extras=True)
            param_strs: list[str] = []
            for pname, _ in sig.parameters.items():
                if pname in ("self", "cls"):
                    continue
                ann = hints.get(pname, Any)
                ann_name = getattr(ann, "__name__", repr(ann))
                param_strs.append(f"{pname}: {ann_name}")
            ret_ann = hints.get("return", Any)
            ret_name = getattr(ret_ann, "__name__", repr(ret_ann))
            rpcs.append(f"{n}({', '.join(param_strs)}) → {ret_name}")

        rpcs = list(boundary_iter(rpcs, " ├─ ", " ├─ ", " └─ "))

        outputs = list(
            boundary_iter(map(str, c.outputs.values()), " ├─ ", " ├─ ", " ├─ " if rpcs else " └─ ")
        )

        if rpcs:
            rpcs = [" │"] + rpcs

        return "\n".join(inputs + [box(c.__name__)] + outputs + rpcs)

    setattr(cls, "io", classmethod(_io_inner))

    # instance method simply forwards to classmethod
    def _io_instance(self):
        return self.__class__.io()

    setattr(cls, "io_instance", _io_instance)

    # Wrap the __init__ method to add print statements
    original_init = cls.__init__

    def wrapped_init(self, *args, **kwargs):
        try:
            self.worker = get_worker()
            self.id = thread_state.key

        except ValueError:
            self.worker = None

        if self.worker:
            print(f"[{cls.__name__}] deployed on worker {self.worker.id} as {self.id}")

        newkwargs = {}

        for k, v in kwargs.items():
            if isinstance(v, Out):
                v.context = self.ref()
                v.inputkey = k
            newkwargs[k] = v

        return original_init(self, *args, **newkwargs)

    cls.__init__ = wrapped_init

    def ref(self) -> ActorReference:
        ref = ActorReference(
            cls=self.__class__,
            workerid=self.worker.address if self.worker else None,
            actorid=self.id if self.id else None,
        )

        # The ref() method gets called on the original instance (on worker)
        # but we need to store the Actor proxy that called it
        # We can detect this by checking if we have thread_state indicating actor execution
        try:
            from distributed.actor import Actor
            from distributed.worker import thread_state

            # Check if we're being called from an actor context
            if hasattr(thread_state, "actor") and thread_state.actor:
                # We're in an actor execution context
                # Create an Actor proxy that points to ourselves
                actor_proxy = Actor(cls=self.__class__, address=self.worker.address, key=self.id)
                ref._actor = actor_proxy
            else:
                pass  # Not in actor context, no need to set _actor
        except Exception:
            pass  # Error checking actor context, continue without setting _actor

        return ref

    cls.ref = ref

    return cls
