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

from dimos.agents.memory.base import AbstractAgentSemanticMemory

import chromadb
from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma
import os
import torch


class ChromaAgentSemanticMemory(AbstractAgentSemanticMemory):
    """Base class for Chroma-based semantic memory implementations."""
    
    def __init__(self, collection_name="my_collection"):
        """Initialize the connection to the local Chroma DB."""
        self.collection_name = collection_name
        self.db_connection = None
        self.embeddings = None
        super().__init__(connection_type='local')

    def connect(self):
        # Stub
        return super().connect()
    
    def create(self):
        """Create the embedding function and initialize the Chroma database.
        This method must be implemented by child classes."""
        raise NotImplementedError("Child classes must implement this method")

    def add_vector(self, vector_id, vector_data):
        """Add a vector to the ChromaDB collection."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")
        self.db_connection.add_texts(
            ids=[vector_id],
            texts=[vector_data],
            metadatas=[{"name": vector_id}],
        )

    def get_vector(self, vector_id):
        """Retrieve a vector from the ChromaDB by its identifier."""
        result = self.db_connection.get(include=['embeddings'], ids=[vector_id])
        return result

    def query(self, query_texts, n_results=4, similarity_threshold=None):
        """Query the collection with a specific text and return up to n results."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")
        
        if similarity_threshold is not None:
            if not (0 <= similarity_threshold <= 1):
                raise ValueError("similarity_threshold must be between 0 and 1.")
            return self.db_connection.similarity_search_with_relevance_scores(
                query=query_texts,
                k=n_results,
                score_threshold=similarity_threshold
            )
        else:
            documents = self.db_connection.similarity_search(
                query=query_texts,
                k=n_results
            )
            return [(doc, None) for doc in documents]

    def update_vector(self, vector_id, new_vector_data):
        # TODO
        return super().connect()

    def delete_vector(self, vector_id):
        """Delete a vector from the ChromaDB using its identifier."""
        if not self.db_connection:
            raise Exception("Collection not initialized. Call connect() first.")
        self.db_connection.delete(ids=[vector_id])


class OpenAISemanticMemory(ChromaAgentSemanticMemory):
    """Semantic memory implementation using OpenAI's embedding API."""
    
    def __init__(self, collection_name="my_collection", model="text-embedding-3-large", dimensions=1024):
        """Initialize OpenAI-based semantic memory.
        
        Args:
            collection_name (str): Name of the Chroma collection
            model (str): OpenAI embedding model to use
            dimensions (int): Dimension of the embedding vectors
        """
        self.model = model
        self.dimensions = dimensions
        super().__init__(collection_name=collection_name)

    def create(self):
        """Connect to OpenAI API and create the ChromaDB client."""
        # Get OpenAI key
        self.OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
        if not self.OPENAI_API_KEY:
            raise Exception("OpenAI key was not specified.")

        # Set embeddings
        self.embeddings = OpenAIEmbeddings(
            model=self.model,
            dimensions=self.dimensions,
            api_key=self.OPENAI_API_KEY,
        )

        # Create the database
        self.db_connection = Chroma(
            collection_name=self.collection_name,
            embedding_function=self.embeddings,
            collection_metadata={"hnsw:space": "cosine"}
        )


class LocalSemanticMemory(ChromaAgentSemanticMemory):
    """Semantic memory implementation using local models."""
    
    def __init__(self, collection_name="my_collection", model_name="sentence-transformers/all-MiniLM-L6-v2"):
        """Initialize the local semantic memory using SentenceTransformer.
        
        Args:
            collection_name (str): Name of the Chroma collection
            model_name (str): Embeddings model
        """
        from sentence_transformers import SentenceTransformer
        
        self.model_name = model_name
        super().__init__(collection_name=collection_name)

    def create(self):
        """Create local embedding model and initialize the ChromaDB client."""
        # Load the sentence transformer model
        # Use CUDA if available, otherwise fall back to CPU
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {device}")
        self.model = SentenceTransformer(self.model_name, device=device)
        
        # Create a custom embedding class that implements the embed_query method
        class SentenceTransformerEmbeddings:
            def __init__(self, model):
                self.model = model
                
            def embed_query(self, text):
                """Embed a single query text."""
                return self.model.encode(text, normalize_embeddings=True).tolist()
                
            def embed_documents(self, texts):
                """Embed multiple documents/texts."""
                return self.model.encode(texts, normalize_embeddings=True).tolist()
        
        # Create an instance of our custom embeddings class
        self.embeddings = SentenceTransformerEmbeddings(self.model)

        # Create the database
        self.db_connection = Chroma(
            collection_name=self.collection_name,
            embedding_function=self.embeddings,
            collection_metadata={"hnsw:space": "cosine"}
        )

