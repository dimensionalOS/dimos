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

"""Unified gateway client for LLM access."""

import asyncio
from collections.abc import AsyncIterator, Iterator
import logging
import os
from types import TracebackType
from typing import Any

import httpx
from tenacity import retry, stop_after_attempt, wait_exponential

from .tensorzero_embedded import TensorZeroEmbeddedGateway

logger = logging.getLogger(__name__)


class UnifiedGatewayClient:
    """Clean abstraction over TensorZero or other gateways.

    This client provides a unified interface for accessing multiple LLM providers
    through a gateway service, with support for streaming, tools, and async operations.
    """

    def __init__(
        self, gateway_url: str | None = None, timeout: float = 60.0, use_simple: bool = False
    ) -> None:
        """Initialize the gateway client.

        Args:
            gateway_url: URL of the gateway service. Defaults to env var or localhost
            timeout: Request timeout in seconds
            use_simple: Deprecated parameter, always uses TensorZero
        """
        self.gateway_url = gateway_url or os.getenv(
            "TENSORZERO_GATEWAY_URL", "http://localhost:3000"
        )
        self.timeout = timeout
        self._client = None
        self._async_client = None

        # Always use TensorZero embedded gateway
        try:
            self._tensorzero_client = TensorZeroEmbeddedGateway()
            logger.info("Using TensorZero embedded gateway")
        except Exception as e:
            logger.error(f"Failed to initialize TensorZero: {e}")
            raise

    def _get_client(self) -> httpx.Client:
        """Get or create sync HTTP client."""
        if self._client is None:
            self._client = httpx.Client(  # type: ignore[assignment]
                base_url=self.gateway_url,  # type: ignore[arg-type]
                timeout=self.timeout,
                headers={"Content-Type": "application/json"},
            )
        return self._client  # type: ignore[return-value]

    def _get_async_client(self) -> httpx.AsyncClient:
        """Get or create async HTTP client."""
        if self._async_client is None:
            self._async_client = httpx.AsyncClient(  # type: ignore[assignment]
                base_url=self.gateway_url,  # type: ignore[arg-type]
                timeout=self.timeout,
                headers={"Content-Type": "application/json"},
            )
        return self._async_client  # type: ignore[return-value]

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def inference(  # type: ignore[no-untyped-def]
        self,
        model: str,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.0,
        max_tokens: int | None = None,
        stream: bool = False,
        **kwargs,
    ) -> dict[str, Any] | Iterator[dict[str, Any]]:
        """Synchronous inference call.

        Args:
            model: Model identifier (e.g., "openai::gpt-4o")
            messages: List of message dicts with role and content
            tools: Optional list of tools in standard format
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            stream: Whether to stream the response
            **kwargs: Additional model-specific parameters

        Returns:
            Response dict or iterator of response chunks if streaming
        """
        return self._tensorzero_client.inference(
            model=model,
            messages=messages,
            tools=tools,
            temperature=temperature,
            max_tokens=max_tokens,
            stream=stream,
            **kwargs,
        )

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    async def ainference(  # type: ignore[no-untyped-def]
        self,
        model: str,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.0,
        max_tokens: int | None = None,
        stream: bool = False,
        **kwargs,
    ) -> dict[str, Any] | AsyncIterator[dict[str, Any]]:
        """Asynchronous inference call.

        Args:
            model: Model identifier (e.g., "anthropic::claude-3-7-sonnet")
            messages: List of message dicts with role and content
            tools: Optional list of tools in standard format
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            stream: Whether to stream the response
            **kwargs: Additional model-specific parameters

        Returns:
            Response dict or async iterator of response chunks if streaming
        """
        return await self._tensorzero_client.ainference(
            model=model,
            messages=messages,
            tools=tools,
            temperature=temperature,
            max_tokens=max_tokens,
            stream=stream,
            **kwargs,
        )

    def close(self) -> None:
        """Close the HTTP clients."""
        if self._client:
            self._client.close()
            self._client = None
        if self._async_client:
            # This needs to be awaited in an async context
            # We'll handle this in __del__ with asyncio
            pass
        self._tensorzero_client.close()

    async def aclose(self) -> None:
        """Async close method."""
        if self._async_client:
            await self._async_client.aclose()
            self._async_client = None
        await self._tensorzero_client.aclose()

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.close()
        if self._async_client:
            # Try to close async client if event loop is available
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    loop.create_task(self.aclose())
                else:
                    loop.run_until_complete(self.aclose())
            except RuntimeError:
                # No event loop, just let it be garbage collected
                pass

    def __enter__(self):  # type: ignore[no-untyped-def]
        """Context manager entry."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Context manager exit."""
        self.close()

    async def __aenter__(self):  # type: ignore[no-untyped-def]
        """Async context manager entry."""
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Async context manager exit."""
        await self.aclose()
