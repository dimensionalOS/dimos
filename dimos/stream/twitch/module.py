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

"""TwitchChat: connects to a Twitch channel and publishes chat messages.

Publishes all messages on ``raw_messages``, and a subset matching regex
patterns on ``filtered_messages``.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass, field
import os
import re
import threading
import time
from typing import Any

try:
    import twitchio  # type: ignore[import-untyped]
    from twitchio.ext import commands as twitch_commands  # type: ignore[import-untyped]
except ImportError:
    raise ImportError(
        "[TwitchChat] The 'twitchio' package is not installed.\n"
        "Install it with:\n"
        "    uv pip install 'twitchio>=2.0,<3.0'\n"
        "or:\n"
        "    pip install 'twitchio>=2.0,<3.0'"
    ) from None

_twitchio_version = getattr(twitchio, "__version__", "unknown")
try:
    _twitchio_major = int(str(_twitchio_version).split(".")[0])
except (ValueError, IndexError):
    _twitchio_major = -1

if _twitchio_major >= 3:
    raise ImportError(
        f"[TwitchChat] twitchio v{_twitchio_version} is installed, but DimOS requires twitchio 2.x.\n"
        "twitchio 3.x has a completely different API and is not compatible.\n"
        "Downgrade with:\n"
        "    uv pip install 'twitchio>=2.0,<3.0'\n"
        "or:\n"
        "    pip install 'twitchio>=2.0,<3.0'"
    )
if _twitchio_major < 2 and _twitchio_major != -1:
    raise ImportError(
        f"[TwitchChat] twitchio v{_twitchio_version} is installed, but DimOS requires twitchio 2.x.\n"
        "Upgrade with:\n"
        "    uv pip install 'twitchio>=2.0,<3.0'\n"
        "or:\n"
        "    pip install 'twitchio>=2.0,<3.0'"
    )

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class TwitchMessage:
    author: str = ""
    content: str = ""
    channel: str = ""
    timestamp: float = 0.0
    is_subscriber: bool = False
    is_mod: bool = False
    badges: dict[str, str] = field(default_factory=dict)

    @property
    def text(self) -> str:
        return self.content

    def find_one(self, options: list[str] | set[str] | frozenset[str]) -> str | None:
        """Return the first option found as a whole word in content (case-insensitive), or None."""
        lower = self.content.lower()
        for opt in options:
            if re.search(rf"\b{re.escape(opt.lower())}\b", lower):
                return opt
        return None

    def __repr__(self) -> str:
        return f"TwitchMessage({self.author}: {self.content!r})"


class TwitchChatConfig(ModuleConfig):
    # OAuth token (oauth:xxx). Falls back to DIMOS_TWITCH_TOKEN / TWITCH_TOKEN env var.
    twitch_token: str = ""
    # Falls back to DIMOS_CHANNEL_NAME / CHANNEL_NAME env var.
    channel_name: str = ""
    bot_prefix: str = "!"
    # Regex patterns for filtered_messages. If empty, all messages pass through.
    patterns: list[str] = []
    # Only pass messages where is_mod matches this value.
    filter_is_mod: bool | None = None
    # Only pass messages where is_subscriber matches this value.
    filter_is_subscriber: bool | None = None
    filter_content: Callable[[str], bool] | None = None
    filter_author: Callable[[str], bool] | None = None
    # When True, messages sent by the bot itself are ignored.
    ignore_self_messages: bool = False


class TwitchChat(Module):
    """Connects to a Twitch channel and publishes chat messages.

    - ``raw_messages`` — every chat message
    - ``filtered_messages`` — messages matching configured regex patterns
    """

    config: TwitchChatConfig

    raw_messages: Out[TwitchMessage]
    filtered_messages: Out[TwitchMessage]

    def __init__(self, **kwargs: Any) -> None:
        self._bot: _TwitchBot | None = None
        self._bot_thread: threading.Thread | None = None
        self._bot_loop: asyncio.AbstractEventLoop | None = None
        self._compiled_patterns: list[re.Pattern[str]] = []
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        token = (
            self.config.twitch_token
            or os.getenv("DIMOS_TWITCH_TOKEN", "")
            or os.getenv("TWITCH_TOKEN", "")
        )
        channel = (
            self.config.channel_name
            or os.getenv("DIMOS_CHANNEL_NAME", "")
            or os.getenv("CHANNEL_NAME", "")
        )

        self._compiled_patterns = [re.compile(p, re.IGNORECASE) for p in self.config.patterns]

        logger.info(
            "[TwitchChat] config.channel_name=%r DIMOS_CHANNEL_NAME=%r CHANNEL_NAME=%r => channel=%s",
            self.config.channel_name,
            os.getenv("DIMOS_CHANNEL_NAME"),
            os.getenv("CHANNEL_NAME"),
            channel,
        )

        missing = [name for name, val in [("token", token), ("channel", channel)] if not val]
        if missing:
            raise RuntimeError(
                f"[TwitchChat] Missing required config: {', '.join(missing)}. "
                "Set via DIMOS_TWITCH_TOKEN / TWITCH_TOKEN and DIMOS_CHANNEL_NAME / CHANNEL_NAME "
                "env vars, or pass twitch_token= / channel_name= to TwitchChat.blueprint()."
            )

        self._bot_loop = asyncio.new_event_loop()
        self._bot_thread = threading.Thread(
            target=self._run_bot_loop,
            args=(token, channel),
            daemon=True,
            name="twitch-bot",
        )
        self._bot_thread.start()
        logger.info("[TwitchChat] Started", channel=channel)

    def _run_bot_loop(self, token: str, channel: str) -> None:
        assert self._bot_loop is not None
        asyncio.set_event_loop(self._bot_loop)
        try:
            self._bot = _TwitchBot(
                token=token,
                channel=channel,
                prefix=self.config.bot_prefix,
                ignore_self_messages=self.config.ignore_self_messages,
                on_message_cb=self._handle_message,
                on_ready_cb=self._handle_ready,
            )
            self._bot.run()
        except Exception:
            logger.exception("[TwitchChat] Bot crashed")
            raise

    @rpc
    def stop(self) -> None:
        if self._bot is not None and self._bot_loop is not None:
            try:

                async def _close() -> None:
                    assert self._bot is not None
                    await self._bot.close()

                asyncio.run_coroutine_threadsafe(_close(), self._bot_loop).result(timeout=5)
            except Exception:
                logger.warning("[TwitchChat] Error closing bot", exc_info=True)

        if self._bot_loop is not None:
            self._bot_loop.call_soon_threadsafe(self._bot_loop.stop)

        if self._bot_thread is not None:
            self._bot_thread.join(timeout=5)

        self._bot = None
        self._bot_thread = None
        self._bot_loop = None
        super().stop()

    def _handle_ready(self) -> None:
        logger.info("[TwitchChat] Ready")

    def _build_twitch_message(self, message: Any) -> TwitchMessage:
        """Convert a raw twitchio Message into a TwitchMessage."""
        badges: dict[str, str] = {}
        if message.tags and "badges" in message.tags:
            raw = message.tags["badges"]
            if raw:
                for badge in raw.split(","):
                    parts = badge.split("/", 1)
                    if len(parts) == 2:
                        badges[parts[0]] = parts[1]

        return TwitchMessage(
            author=message.author.name if message.author else "",
            content=message.content or "",
            channel=message.channel.name if message.channel else "",
            timestamp=time.time(),
            is_subscriber="subscriber" in badges,
            is_mod="moderator" in badges,
            badges=badges,
        )

    def _handle_message(self, message: Any) -> None:
        msg = self._build_twitch_message(message)
        self.raw_messages.publish(msg)
        self._publish_if_matched(msg)
        self._on_message_received(msg)

    def _on_message_received(self, msg: TwitchMessage) -> None:
        """Hook for subclasses to process messages after publishing."""

    def _publish_if_matched(self, msg: TwitchMessage) -> None:
        """Publish to filtered_messages if msg passes all configured filters."""
        cfg = self.config

        if cfg.filter_is_mod is not None and msg.is_mod != cfg.filter_is_mod:
            return
        if cfg.filter_is_subscriber is not None and msg.is_subscriber != cfg.filter_is_subscriber:
            return
        if cfg.filter_author is not None and not cfg.filter_author(msg.author):
            return
        if cfg.filter_content is not None and not cfg.filter_content(msg.content):
            return

        if self._compiled_patterns:
            for pat in self._compiled_patterns:
                if pat.search(msg.content):
                    self.filtered_messages.publish(msg)
                    return
        else:
            self.filtered_messages.publish(msg)

    def inject_message(self, content: str, author: str = "anonymous") -> None:
        """Inject a chat message programmatically (for testing or local-only mode)."""
        msg = TwitchMessage(author=author, content=content, channel="local", timestamp=time.time())
        self.raw_messages.publish(msg)
        self._publish_if_matched(msg)
        self._on_message_received(msg)


class _TwitchBot:
    """Thin twitchio wrapper that forwards messages via callbacks."""

    def __init__(
        self,
        token: str,
        channel: str,
        prefix: str,
        ignore_self_messages: bool,
        on_message_cb: Any,
        on_ready_cb: Any,
    ) -> None:
        cb_message = on_message_cb
        cb_ready = on_ready_cb
        chan = channel
        skip_echo = ignore_self_messages

        class _Bot(twitch_commands.Bot):  # type: ignore[misc]
            def __init__(inner_self) -> None:  # noqa: N805
                super().__init__(token=token, prefix=prefix, initial_channels=[chan])

            async def event_ready(inner_self) -> None:  # noqa: N805
                logger.info("[TwitchChat] Bot connected as %s to #%s", inner_self.nick, chan)
                cb_ready()

            async def event_message(inner_self, message: Any) -> None:  # noqa: N805
                if skip_echo and message.echo:
                    return
                cb_message(message)

        self._bot = _Bot()

    def run(self) -> None:
        self._bot.run()

    async def close(self) -> None:
        await self._bot.close()
