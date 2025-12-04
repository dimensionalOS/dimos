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

from collections.abc import Callable
from typing import Any

from dimos.protocol.rpc import LCMRPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RpcCall:
    _original_method: Callable[..., Any] | None
    _rpc: LCMRPC | None
    _name: str
    _remote_name: str
    _unsub_fns: list  # type: ignore[type-arg]
    _stop_rpc_client: Callable[[], None] | None = None

    def __init__(
        self,
        original_method: Callable[..., Any] | None,
        rpc: LCMRPC,
        name: str,
        remote_name: str,
        unsub_fns: list,  # type: ignore[type-arg]
        stop_client: Callable[[], None] | None = None,
    ) -> None:
        self._original_method = original_method
        self._rpc = rpc
        self._name = name
        self._remote_name = remote_name
        self._unsub_fns = unsub_fns
        self._stop_rpc_client = stop_client

        if original_method:
            self.__doc__ = original_method.__doc__
            self.__name__ = original_method.__name__
            self.__qualname__ = f"{self.__class__.__name__}.{original_method.__name__}"

    def set_rpc(self, rpc: LCMRPC) -> None:
        self._rpc = rpc

    def __call__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        if not self._rpc:
            logger.warning("RPC client not initialized")
            return None

        # For stop, use call_nowait to avoid deadlock
        # (the remote side stops its RPC service before responding)
        if self._name == "stop":
            self._rpc.call_nowait(f"{self._remote_name}/{self._name}", (args, kwargs))  # type: ignore[arg-type]
            if self._stop_rpc_client:
                self._stop_rpc_client()
            return None

        result, unsub_fn = self._rpc.call_sync(f"{self._remote_name}/{self._name}", (args, kwargs))  # type: ignore[arg-type]
        self._unsub_fns.append(unsub_fn)
        return result

    def __getstate__(self):  # type: ignore[no-untyped-def]
        return (self._original_method, self._name, self._remote_name)

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self._original_method, self._name, self._remote_name = state
        self._unsub_fns = []
        self._rpc = None
        self._stop_rpc_client = None


class RPCClient:
    def __init__(self, actor_instance, actor_class) -> None:  # type: ignore[no-untyped-def]
        self.rpc = LCMRPC()
        self.actor_class = actor_class
        self.remote_name = actor_class.__name__
        self.actor_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()
        self.rpc.start()
        self._unsub_fns = []  # type: ignore[var-annotated]

    def stop_rpc_client(self) -> None:
        for unsub in self._unsub_fns:
            try:
                unsub()
            except Exception:
                pass

        self._unsub_fns = []

        if self.rpc:
            self.rpc.stop()
            self.rpc = None  # type: ignore[assignment]

    def __reduce__(self):  # type: ignore[no-untyped-def]
        # Return the class and the arguments needed to reconstruct the object
        return (
            self.__class__,
            (self.actor_instance, self.actor_class),
        )

    # passthrough
    def __getattr__(self, name: str):  # type: ignore[no-untyped-def]
        # Check if accessing a known safe attribute to avoid recursion
        if name in {
            "__class__",
            "__init__",
            "__dict__",
            "__getattr__",
            "rpcs",
            "remote_name",
            "remote_instance",
            "actor_instance",
        }:
            raise AttributeError(f"{name} is not found.")

        if name in self.rpcs:
            original_method = getattr(self.actor_class, name, None)
            return RpcCall(
                original_method,
                self.rpc,
                name,
                self.remote_name,
                self._unsub_fns,
                self.stop_rpc_client,
            )

        # return super().__getattr__(name)
        # Try to avoid recursion by directly accessing attributes that are known
        return self.actor_instance.__getattr__(name)
