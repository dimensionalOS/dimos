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

from collections.abc import Callable
from contextlib import ExitStack
import inspect
import os
import pickle
import threading
import time
from typing import Any, Protocol
from unittest.mock import patch

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.protocol.rpc.spec import Args
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.spec.utils import Spec


def debug_rpc(stage: str, **fields: Any) -> None:
    """Flush each event so logs from both worker processes appear immediately."""
    details = " ".join(f"{key}={value!r}" for key, value in fields.items())
    print(
        f"[RPC DEBUG {time.time():.6f} pid={os.getpid()} "
        f"thread={threading.current_thread().name}] {stage}: {details}",
        flush=True,
    )


# this would be defined in some other file (this could be imported from a library)
class Calculator(Module):
    @rpc
    def compute1(self, a: int, b: int) -> int:
        debug_rpc("4 SERVER execute", method="compute1", a=a, b=b)
        result = a + b
        debug_rpc("5 SERVER return to RPC encoder", method="compute1", result=result)
        return result

    @rpc
    def compute2(self, a: float, b: float) -> float:
        debug_rpc("4 SERVER execute", method="compute2", a=a, b=b)
        result = a + b
        debug_rpc("5 SERVER return to RPC encoder", method="compute2", result=result)
        return result


# what your module needs/expects
class ComputeSpec(Spec, Protocol):
    @rpc
    def compute1(self, a: int, b: int) -> int: ...

    @rpc
    def compute2(self, a: float, b: float) -> float: ...


class Client(Module):
    # this says: "hey dimos, give me access to a module that has a compute1 and compute2 method"
    calc: ComputeSpec

    @rpc
    def start(self) -> None:
        super().start()
        proxy = self.calc
        if not isinstance(proxy, RPCClient):
            raise TypeError(f"Expected an injected RPCClient, got {type(proxy).__qualname__}")
        transport = proxy.rpc
        debug_rpc(
            "0 CLIENT injected Spec reference",
            declared_spec=ComputeSpec.__qualname__,
            actual_type=f"{type(proxy).__module__}.{type(proxy).__qualname__}",
            remote_name=proxy.remote_name,
            advertised_rpcs=sorted(proxy.rpcs),
            actor_class=proxy.actor_class,
            note="actor_class may be None after the proxy crosses a worker boundary",
        )
        debug_rpc(
            "0 CLIENT transport",
            backend=f"{type(transport).__module__}.{type(transport).__qualname__}",
            default_timeout_s=transport.default_rpc_timeout,
            method_timeouts_s=transport.rpc_timeouts,
            flow="RpcCall -> call_sync -> call -> encode/send -> server -> decode/callback -> return",
        )

        original_call = transport.call

        def traced_call(
            name: str, arguments: Args, cb: Callable[[Any], None] | None
        ) -> Callable[[], Any] | None:
            started = time.perf_counter()
            debug_rpc(
                "2 CLIENT RPC dispatch",
                name=name,
                args=arguments[0],
                kwargs=arguments[1],
                expects_response=cb is not None,
            )
            if cb is None:
                return original_call(name, arguments, None)

            def receive_result(value: Any) -> None:
                debug_rpc(
                    "6 CLIENT decoded response callback",
                    name=name,
                    result=value,
                    result_type=type(value).__qualname__,
                    is_exception=isinstance(value, BaseException),
                    elapsed_ms=(time.perf_counter() - started) * 1000,
                    note="response already decoded by the backend; callback wakes call_sync",
                )
                cb(value)

            return original_call(name, arguments, receive_result)

        # Debug-only probes on this proxy's transport; restore even if a call fails.
        with ExitStack() as probes:
            probes.enter_context(patch.object(transport, "call", new=traced_call))
            if isinstance(transport, ZenohRPC):
                original_issue_query = transport._issue_query

                def traced_query(call_id: int, key: str, payload: bytes, deadline: float) -> None:
                    debug_rpc(
                        "3 CLIENT actual Zenoh request (also logged on retry)",
                        call_id=call_id,
                        key=key,
                        encoding="pickle",
                        payload_bytes=len(payload),
                        payload_hex=payload.hex(),
                        decoded_request=pickle.loads(payload),
                        remaining_timeout_s=max(0.0, deadline - time.monotonic()),
                        note="call_id is local callback bookkeeping, not part of the payload",
                    )
                    original_issue_query(call_id, key, payload, deadline)

                probes.enter_context(patch.object(transport, "_issue_query", new=traced_query))
            else:
                debug_rpc("3 WIRE probe unavailable", note="raw payload tracing supports ZenohRPC")

            calls: tuple[tuple[str, tuple[int | float, ...]], ...] = (
                ("compute1", (2, 3)),
                ("compute2", (1.5, 2.5)),
            )
            for method_name, args in calls:
                method = getattr(proxy, method_name)
                if not isinstance(method, RpcCall):
                    raise TypeError(f"Expected RpcCall for {method_name}, got {type(method)}")
                debug_rpc(
                    "1 CLIENT call remote method",
                    callable_type=type(method).__qualname__,
                    remote_name=method.remote_name,
                    rpc_name=method.rpc_name,
                    declared_signature=str(inspect.signature(getattr(ComputeSpec, method_name))),
                    args=args,
                )
                started = time.perf_counter()
                try:
                    result = method(*args)
                except Exception as exc:
                    debug_rpc("7 CLIENT raised", method=method_name, exception=repr(exc))
                    raise
                debug_rpc(
                    "7 CLIENT call returned",
                    method=method_name,
                    result=result,
                    result_type=type(result).__qualname__,
                    elapsed_ms=(time.perf_counter() - started) * 1000,
                )
        debug_rpc("8 CLIENT debug probes restored")


if __name__ == "__main__":
    ModuleCoordinator.build(
        autoconnect(
            Calculator.blueprint(),
            Client.blueprint(),
        )
    ).loop()
