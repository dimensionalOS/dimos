# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""LLM factory.

Thin wrapper over LangChain's ``init_chat_model`` so callers can pass
either a bare model name (``"gpt-4.1-mini"``, ``"gemini-2.5-flash"``) or
a fully qualified ``"<provider>:<model>"`` string. Gemini models also
get an in-process rate limiter, since the public free tier is RPM-capped.
"""

from __future__ import annotations

import os

from langchain.chat_models import init_chat_model
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.rate_limiters import InMemoryRateLimiter


def _gemini_rate_limiter() -> InMemoryRateLimiter:
    rpm = float(os.environ.get("GEMINI_RPM_LIMIT", "12"))
    return InMemoryRateLimiter(
        requests_per_second=rpm / 60.0,
        check_every_n_seconds=0.1,
        max_bucket_size=1,
    )


def build_chat_model(model: str, temperature: float = 0.0) -> BaseChatModel:
    # Strip an optional "provider:" prefix to detect gemini for the rate
    # limiter; init_chat_model handles the actual provider routing.
    bare = model.split(":", 1)[-1]
    kwargs: dict = {"temperature": temperature, "max_retries": 4}
    if bare.startswith("gemini"):
        kwargs["rate_limiter"] = _gemini_rate_limiter()
        kwargs["max_retries"] = 6
    return init_chat_model(model, **kwargs)
