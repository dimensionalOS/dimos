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
import functools
import inspect
import threading
from typing import Any, TypeVar, cast

from dimos.core.core import rpc

F = TypeVar("F", bound=Callable[..., Any])

_SKILL_CONTEXT = threading.local()


def current_skill_context() -> dict[str, Any] | None:
    """Return the per-call context for the currently executing `@skill`.

    Returns a (possibly empty) dict inside a `@skill` call and `None` when no
    skill is currently on the stack in this thread. The MCP server populates
    `{"progress_token": <token>}` when the caller supplied
    `params._meta.progressToken`. Otherwise the dict is empty. Downstream code
    uses the `None` vs. `{}` distinction to tell "outside any skill" from
    "inside a skill that didn't get a token."
    """
    return getattr(_SKILL_CONTEXT, "context", None)


def skill(func: F) -> F:
    if inspect.iscoroutinefunction(func):

        @functools.wraps(func)
        async def async_context_wrapper(*args: Any, **kwargs: Any) -> Any:
            context = kwargs.pop("_mcp_context", None) or {}
            previous = getattr(_SKILL_CONTEXT, "context", None)
            _SKILL_CONTEXT.context = context
            try:
                return await func(*args, **kwargs)
            finally:
                _SKILL_CONTEXT.context = previous

        context_wrapper: Callable[..., Any] = async_context_wrapper
    else:

        @functools.wraps(func)
        def sync_context_wrapper(*args: Any, **kwargs: Any) -> Any:
            context = kwargs.pop("_mcp_context", None) or {}
            previous = getattr(_SKILL_CONTEXT, "context", None)
            _SKILL_CONTEXT.context = context
            try:
                return func(*args, **kwargs)
            finally:
                _SKILL_CONTEXT.context = previous

        context_wrapper = sync_context_wrapper

    wrapped = rpc(context_wrapper)
    wrapped.__skill__ = True  # type: ignore[attr-defined]
    return cast("F", wrapped)
