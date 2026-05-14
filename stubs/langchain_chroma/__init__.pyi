"""langchain_chroma is ~310 MB, so we have a stub to avoid installing in lint job."""

from collections.abc import Iterable
from typing import Any, Protocol

class _Embeddings(Protocol):
    def embed_query(self, text: str) -> list[float]: ...
    def embed_documents(self, texts: Iterable[str]) -> list[list[float]]: ...

class Chroma:
    def __init__(
        self,
        collection_name: str = ...,
        embedding_function: _Embeddings | None = ...,
        persist_directory: str | None = ...,
        client_settings: Any | None = ...,
        collection_metadata: dict[str, Any] | None = ...,
        client: Any | None = ...,
        relevance_score_fn: Any | None = ...,
    ) -> None: ...
    def similarity_search(
        self, query: str, k: int = ..., filter: dict[str, Any] | None = ...
    ) -> list[Any]: ...
    def similarity_search_with_score(
        self, query: str, k: int = ..., filter: dict[str, Any] | None = ...
    ) -> list[tuple[Any, float]]: ...
    def add_texts(
        self,
        texts: Iterable[str],
        metadatas: list[dict[str, Any]] | None = ...,
        ids: list[str] | None = ...,
    ) -> list[str]: ...
    def delete(self, ids: list[str] | None = ..., **kwargs: Any) -> bool | None: ...
