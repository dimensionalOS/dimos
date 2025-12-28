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
import functools
import time
from typing import Any, TypeVar, cast

from dimos_lcm.std_msgs import Float32  # type: ignore[import-untyped]

from dimos.core import LCMTransport, Transport

F = TypeVar("F", bound=Callable[..., Any])


def timed(
    transport: Callable[[F], Transport[Float32]] | Transport[Float32] | None = None,
) -> Callable[[F], F]:
    def timed_decorator(func: F) -> F:
        t: Transport[Float32]
        if transport is None:
            t = LCMTransport(f"/metrics/{func.__name__}", Float32)
        elif callable(transport):
            t = transport(func)
        else:
            t = transport

        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter()
            result = func(*args, **kwargs)
            elapsed = time.perf_counter() - start

            msg = Float32()
            msg.data = elapsed * 1000  # ms
            t.publish(msg)
            return result

        return cast("F", wrapper)

    return timed_decorator
