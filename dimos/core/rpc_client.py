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


from dimos.protocol.rpc.lcmrpc import LCMRPC


class RpcCall:
    def __init__(self, original_method, rpc, name, remote_name, unsub_fns):
        self.original_method = original_method
        self.rpc = rpc
        self.name = name
        self.remote_name = remote_name
        self._unsub_fns = unsub_fns

        if original_method:
            self.__doc__ = original_method.__doc__
            self.__name__ = original_method.__name__
            self.__qualname__ = f"{self.__class__.__name__}.{original_method.__name__}"

    def __call__(self, *args, **kwargs):
        # For stop/close/shutdown, use call_nowait to avoid deadlock
        # (the remote side stops its RPC service before responding)
        if self.name in ("stop"):
            if self.rpc:
                self.rpc.call_nowait(f"{self.remote_name}/{self.name}", (args, kwargs))
            self.stop_client()
            return None

        result, unsub_fn = self.rpc.call_sync(f"{self.remote_name}/{self.name}", (args, kwargs))
        print(
            f"RPC call to {self.remote_name}/{self.name} with args={args}, kwargs={kwargs}",
            self.rpc,
            result,
        )
        self._unsub_fns.append(unsub_fn)
        return result

    def __getstate__(self):
        return (self.original_method, self.name, self.remote_name, self._unsub_fns)

    def __setstate__(self, state):
        self.original_method, self.name, self.remote_name, self._unsub_fns = state


class RPCClient:
    def __init__(self, actor_instance, actor_class):
        self.rpc = LCMRPC()
        self.actor_class = actor_class
        self.remote_name = actor_class.__name__
        self.actor_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()
        self.rpc.start()
        self._unsub_fns = []

    def stop_client(self):
        for unsub in self._unsub_fns:
            try:
                unsub()
            except Exception:
                pass

        self._unsub_fns = []

        if self.rpc:
            self.rpc.stop()
            self.rpc = None

    def __reduce__(self):
        # Return the class and the arguments needed to reconstruct the object
        return (
            self.__class__,
            (self.actor_instance, self.actor_class),
        )

    # passthrough
    def __getattr__(self, name: str):
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
            return RpcCall(original_method, self.rpc, name, self.remote_name, self._unsub_fns)

        # return super().__getattr__(name)
        # Try to avoid recursion by directly accessing attributes that are known
        return self.actor_instance.__getattr__(name)
