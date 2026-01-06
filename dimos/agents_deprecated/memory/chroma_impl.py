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

from collections.abc import Sequence
import os

from langchain_chroma import Chroma
from langchain_openai import OpenAIEmbeddings
import torch

from dimos.agents_deprecated.memory.base import AbstractAgentSemanticMemory


class ChromaAgentSemanticMemory(AbstractAgentSemanticMemory):
    """Base class for Chroma-based semantic memory implementations."""

    def __init__(self, collection_name: str = "my_collection") -> None:
        """Initialize the connection to the local Chroma DB."""
        self.collection_name = collection_name
        self.db_connection = None
        self.embeddings = None
        super().__init__(connection_type="local")

    def connect(self):  # type: ignore[no-untyped-def]
        # Stub
        return super().connect()  # type: ignore[no-untyped-call, safe-super]

    def create(self):  # type: ignore[no-untyped-def]
        """Create the embedding function and initialize the Chroma database.
        This method must be implemented by child classes."""
        raise NotImplementedError("Child classes must implement this method")

    def add_vector(self, vector_id, vector_data):  # type: ignore[no-untyped-def]
        """Add a vector to the ChromaDB collection."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")
        self.db_connection.add_texts(
            ids=[vector_id],
            texts=[vector_data],
            metadatas=[{"name": vector_id}],
        )

    def get_vector(self, vector_id):  # type: ignore[no-untyped-def]
        """Retrieve a vector from the ChromaDB by its identifier."""
        result = self.db_connection.get(include=["embeddings"], ids=[vector_id])  # type: ignore[attr-defined]
        return result

    def query(self, query_texts, n_results: int = 4, similarity_threshold=None):  # type: ignore[no-untyped-def]
        """Query the collection with a specific text and return up to n results."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")

        if similarity_threshold is not None:
            if not (0 <= similarity_threshold <= 1):
                raise ValueError("similarity_threshold must be between 0 and 1.")
            return self.db_connection.similarity_search_with_relevance_scores(
                query=query_texts, k=n_results, score_threshold=similarity_threshold
            )
        else:
            documents = self.db_connection.similarity_search(query=query_texts, k=n_results)
            return [(doc, None) for doc in documents]

    def update_vector(self, vector_id, new_vector_data):  # type: ignore[no-untyped-def]
        # TODO
        return super().connect()  # type: ignore[no-untyped-call, safe-super]

    def delete_vector(self, vector_id):  # type: ignore[no-untyped-def]
        """Delete a vector from the ChromaDB using its identifier."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")
        self.db_connection.delete(ids=[vector_id])


class OpenAISemanticMemory(ChromaAgentSemanticMemory):
    """Semantic memory implementation using OpenAI's embedding API."""

    def __init__(
        self,
        collection_name: str = "my_collection",
        model: str = "text-embedding-3-large",
        dimensions: int = 1024,
    ) -> None:
        """Initialize OpenAI-based semantic memory.

        Args:
            collection_name (str): Name of the Chroma collection
            model (str): OpenAI embedding model to use
            dimensions (int): Dimension of the embedding vectors
        """
        self.model = model
        self.dimensions = dimensions
        super().__init__(collection_name=collection_name)

    def create(self):  # type: ignore[no-untyped-def]
        """Connect to OpenAI API and create the ChromaDB client."""
        # Get OpenAI key
        self.OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
        if not self.OPENAI_API_KEY:
            raise Exception("OpenAI key was not specified.")

        # Set embeddings
        self.embeddings = OpenAIEmbeddings(  # type: ignore[assignment]
            model=self.model,
            dimensions=self.dimensions,
            api_key=self.OPENAI_API_KEY,  # type: ignore[arg-type]
        )

        # Create the database
        self.db_connection = Chroma(  # type: ignore[assignment]
            collection_name=self.collection_name,
            embedding_function=self.embeddings,
            collection_metadata={"hnsw:space": "cosine"},
        )


class LocalSemanticMemory(ChromaAgentSemanticMemory):
    """Semantic memory implementation using local models."""

    def __init__(
        self,
        collection_name: str = "my_collection",
        model_name: str = "sentence-transformers/all-MiniLM-L6-v2",
    ) -> None:
        """Initialize the local semantic memory using SentenceTransformer.

        Args:
            collection_name (str): Name of the Chroma collection
            model_name (str): Embeddings model
        """

        self.model_name = model_name
        super().__init__(collection_name=collection_name)

    def create(self) -> None:
        """Create local embedding model and initialize the ChromaDB client."""
        # Load the sentence transformer model

        # Use GPU if available, otherwise fall back to CPU
        if torch.cuda.is_available():
            self.device = "cuda"
        # MacOS Metal performance shaders
        elif torch.backends.mps.is_available() and torch.backends.mps.is_built():
            self.device = "mps"
        else:
            self.device = "cpu"

        print(f"Using device: {self.device}")
        self.model = SentenceTransformer(self.model_name, device=self.device)  # type: ignore[name-defined]

        # Create a custom embedding class that implements the embed_query method
        class SentenceTransformerEmbeddings:
            def __init__(self, model) -> None:  # type: ignore[no-untyped-def]
                self.model = model

            def embed_query(self, text: str):  # type: ignore[no-untyped-def]
                """Embed a single query text."""
                return self.model.encode(text, normalize_embeddings=True).tolist()

            def embed_documents(self, texts: Sequence[str]):  # type: ignore[no-untyped-def]
                """Embed multiple documents/texts."""
                return self.model.encode(texts, normalize_embeddings=True).tolist()

        # Create an instance of our custom embeddings class
        self.embeddings = SentenceTransformerEmbeddings(self.model)  # type: ignore[assignment]

        # Create the database
        self.db_connection = Chroma(  # type: ignore[assignment]
            collection_name=self.collection_name,
            embedding_function=self.embeddings,
            collection_metadata={"hnsw:space": "cosine"},
        )
