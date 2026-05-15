from collections.abc import Iterable, Mapping
from typing import Any

from .config import Settings as Settings

class Collection:
    name: str
    def add(
        self,
        ids: list[str],
        documents: list[str] | None = ...,
        embeddings: list[list[float]] | None = ...,
        metadatas: list[Mapping[str, Any]] | None = ...,
    ) -> None: ...
    def get(
        self,
        ids: list[str] | None = ...,
        where: Mapping[str, Any] | None = ...,
        limit: int | None = ...,
        offset: int | None = ...,
        include: Iterable[str] | None = ...,
    ) -> dict[str, Any]: ...
    def query(
        self,
        query_embeddings: list[list[float]] | None = ...,
        query_texts: list[str] | None = ...,
        n_results: int = ...,
        where: Mapping[str, Any] | None = ...,
        include: Iterable[str] | None = ...,
    ) -> dict[str, Any]: ...
    def delete(
        self,
        ids: list[str] | None = ...,
        where: Mapping[str, Any] | None = ...,
    ) -> None: ...
    def count(self) -> int: ...

class _ClientBase:
    def list_collections(self) -> list[Collection]: ...
    def get_collection(self, name: str) -> Collection: ...
    def create_collection(
        self, name: str, metadata: Mapping[str, Any] | None = ...
    ) -> Collection: ...
    def get_or_create_collection(
        self, name: str, metadata: Mapping[str, Any] | None = ...
    ) -> Collection: ...
    def delete_collection(self, name: str) -> None: ...

class Client(_ClientBase):
    def __init__(self, settings: Settings | None = ...) -> None: ...

class PersistentClient(_ClientBase):
    def __init__(self, path: str = ..., settings: Settings | None = ...) -> None: ...

class HttpClient(_ClientBase):
    def __init__(
        self, host: str = ..., port: int = ..., settings: Settings | None = ...
    ) -> None: ...
