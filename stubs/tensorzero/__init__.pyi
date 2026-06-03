"""Tensorzero is ~40MB, so we have a stub to avoid installing in lint job."""

from collections.abc import Awaitable
from typing import TypeVar

_T = TypeVar("_T")

def patch_openai_client(
    client: _T,
    *,
    config_file: str | None = ...,
    clickhouse_url: str | None = ...,
    async_setup: bool = ...,
) -> _T | Awaitable[_T]: ...
