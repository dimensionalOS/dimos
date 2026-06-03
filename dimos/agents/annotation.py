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
from dataclasses import replace
import functools
import inspect
import threading
import time
from typing import Any, Literal, TypeVar, cast, overload

from dimos.core.core import rpc
from dimos.utils.logging_config import setup_logger

F = TypeVar("F", bound=Callable[..., Any])

logger = setup_logger()

_SKILL_CONTEXT = threading.local()

SkillLifecycle = Literal["instant", "background"]
_VALID_LIFECYCLES = ("instant", "background")


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


def _stamp_and_log(func_name: str, result: Any, elapsed_ms: float) -> Any:
    """If ``result`` is a ``SkillResult``, attach the elapsed duration and log.

    Returns the (possibly new) result. Skills returning non-SkillResult values
    still get logged with the duration, but no result mutation happens.
    """
    # Lazy import to avoid a hard dependency cycle on package init.
    from dimos.agents.skill_result import SkillResult

    if isinstance(result, SkillResult):
        result = replace(result, duration_ms=elapsed_ms)
        if result.success:
            code = "OK"
        else:
            # success=False is authoritative; error_code may be unset.
            code = result.error_code if result.error_code is not None else "FAILED"
    else:
        # Not a SkillResult — we can't verify the outcome, so don't claim "OK".
        code = "UNKNOWN"
    logger.info("SKILL %s result=%s duration_ms=%.1f", func_name, code, elapsed_ms)
    return result


def _make_skill(func: F, uses: list[str], lifecycle: SkillLifecycle) -> F:
    if inspect.iscoroutinefunction(func):

        @functools.wraps(func)
        async def async_context_wrapper(*args: Any, **kwargs: Any) -> Any:
            context = kwargs.pop("_mcp_context", None) or {}
            previous = getattr(_SKILL_CONTEXT, "context", None)
            _SKILL_CONTEXT.context = context
            t0 = time.monotonic()
            try:
                result = await func(*args, **kwargs)
            except BaseException:
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                logger.info("SKILL %s result=EXCEPTION duration_ms=%.1f", func.__name__, elapsed_ms)
                raise
            finally:
                _SKILL_CONTEXT.context = previous
            return _stamp_and_log(func.__name__, result, (time.monotonic() - t0) * 1000.0)

        context_wrapper: Callable[..., Any] = async_context_wrapper
    else:

        @functools.wraps(func)
        def sync_context_wrapper(*args: Any, **kwargs: Any) -> Any:
            context = kwargs.pop("_mcp_context", None) or {}
            previous = getattr(_SKILL_CONTEXT, "context", None)
            _SKILL_CONTEXT.context = context
            t0 = time.monotonic()
            try:
                result = func(*args, **kwargs)
            except BaseException:
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                logger.info("SKILL %s result=EXCEPTION duration_ms=%.1f", func.__name__, elapsed_ms)
                raise
            finally:
                _SKILL_CONTEXT.context = previous
            return _stamp_and_log(func.__name__, result, (time.monotonic() - t0) * 1000.0)

        context_wrapper = sync_context_wrapper

    wrapped = rpc(context_wrapper)
    wrapped.__skill__ = True  # type: ignore[attr-defined]
    wrapped.__skill_uses__ = list(uses)  # type: ignore[attr-defined]
    wrapped.__skill_lifecycle__ = lifecycle  # type: ignore[attr-defined]
    return cast("F", wrapped)


@overload
def skill(func: F) -> F: ...
@overload
def skill(*, uses: list[str] | None = ..., lifecycle: SkillLifecycle = ...) -> Callable[[F], F]: ...


def skill(
    func: F | None = None,
    *,
    uses: list[str] | None = None,
    lifecycle: SkillLifecycle = "instant",
) -> F | Callable[[F], F]:
    """Mark a method as an agent-callable skill.

    Supports both bare-form `@skill` and parameterized `@skill(uses=[...], lifecycle=...)`.

    `uses` declares capabilities the skill needs (e.g. `["movement"]`). The MCP
    server uses these to refuse the call when another skill is already holding
    a required capability. Default: no capabilities.

    `lifecycle` is `"instant"` (default) for skills that finish their work before
    returning, or `"background"` for skills that kick off background work and
    return early -- those must use `start_tool`/`stop_tool` so the matching
    stop-tool frame can release their capabilities.
    """
    if lifecycle not in _VALID_LIFECYCLES:
        raise ValueError(f"lifecycle must be one of {_VALID_LIFECYCLES}, got {lifecycle!r}")
    if func is not None:
        return _make_skill(func, uses=[], lifecycle="instant")

    def decorator(f: F) -> F:
        return _make_skill(f, uses=list(uses or []), lifecycle=lifecycle)

    return decorator
