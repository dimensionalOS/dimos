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

from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, TypeVar, cast

from dimos.core.resource import CompositeResource
from dimos.memory2.stream import Stream
from dimos.protocol.service.spec import Configurable

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.backend import Backend, BlobStore, LiveChannel, VectorStore

T = TypeVar("T")


# ── Configuration ─────────────────────────────────────────────────


@dataclass
class StoreConfig:
    """Base config for Store. Subclasses extend with store-specific fields."""


@dataclass
class SessionConfig:
    """Session-level defaults for stream capabilities.

    These are inherited by all streams in the session unless overridden
    per-stream in ``session.stream(..., **overrides)``.
    """

    live_channel: LiveChannel[Any] | None = None
    blob_store: BlobStore | None = None
    vector_store: VectorStore | None = None


# ── Stream namespace ──────────────────────────────────────────────


class StreamNamespace:
    """Attribute-access proxy for session streams.

    Usage::

        session.streams.image_stream
        session.streams["image_stream"]
        list(session.streams)
        len(session.streams)
    """

    def __init__(self, session: Session) -> None:
        self._session = session

    def __getattr__(self, name: str) -> Stream[Any]:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._session.list_streams():
            available = ", ".join(self._session.list_streams()) or "(none)"
            raise AttributeError(f"No stream named {name!r}. Available: {available}")
        return self._session.stream(name)

    def __getitem__(self, name: str) -> Stream[Any]:
        if name not in self._session.list_streams():
            raise KeyError(name)
        return self._session.stream(name)

    def __iter__(self) -> Iterator[Stream[Any]]:
        for name in self._session.list_streams():
            yield self._session.stream(name)

    def __len__(self) -> int:
        return len(self._session.list_streams())

    def __contains__(self, name: str) -> bool:
        return name in self._session.list_streams()

    def __repr__(self) -> str:
        return f"StreamNamespace({self._session.list_streams()})"


# ── Session & Store ───────────────────────────────────────────────


class Session(Configurable[SessionConfig], CompositeResource):
    """A session against a store. Manages named streams over a shared connection.

    Subclasses implement ``_create_backend`` to provide storage-specific backends.
    """

    default_config: type[SessionConfig] = SessionConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)
        self._streams: dict[str, Stream[Any]] = {}

    @abstractmethod
    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        """Create a backend for the named stream. Called once per stream name."""
        ...

    def stream(self, name: str, payload_type: type[T] | None = None, **overrides: Any) -> Stream[T]:
        """Get or create a named stream. Returns the same Stream on repeated calls.

        Per-stream ``overrides`` (e.g. ``live_channel=``) are merged on top of
        the session-level defaults from :class:`SessionConfig`.
        """
        if name not in self._streams:
            resolved = {k: v for k, v in vars(self.config).items() if v is not None}
            resolved.update({k: v for k, v in overrides.items() if v is not None})
            backend = self._create_backend(name, payload_type, **resolved)
            self._streams[name] = Stream(source=backend)
        return cast("Stream[T]", self._streams[name])

    @abstractmethod
    def list_streams(self) -> list[str]:
        """Return names of all streams in this session."""
        ...

    @abstractmethod
    def delete_stream(self, name: str) -> None:
        """Delete a stream by name (from cache and underlying storage)."""
        ...

    @property
    def streams(self) -> StreamNamespace:
        return StreamNamespace(self)


class Store(Configurable[StoreConfig], CompositeResource):
    """Top-level entry point — wraps a storage location (file, URL, etc.)."""

    default_config: type[StoreConfig] = StoreConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)

    @abstractmethod
    def session(self, **kwargs: Any) -> Session:
        """Create a session. kwargs are forwarded to SessionConfig."""
        ...
