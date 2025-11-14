from dimos.agents.memory.base import AbstractAgentSemanticMemory

import chromadb
from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma
import os

class AgentSemanticMemory(AbstractAgentSemanticMemory):
    def __init__(self, collection_name="my_collection"):
        """Initialize the connection to the local Chroma DB."""
        self.collection_name = collection_name
        super().__init__(connection_type='local')

    def connect(self):
        # Stub
        return super().connect()

    def create(self):
        """Connect locally, creating the ChromaDB client."""

        # Get OpenAI key
        self.OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
        if not self.OPENAI_API_KEY:
            raise Exception("OpenAI key was not specified.")

        # Set embeddings
        self.embeddings = OpenAIEmbeddings(
            model="text-embedding-3-large",
            dimensions=1024,
            api_key=self.OPENAI_API_KEY,
        )

        # Create the local database
        self.db_connection = Chroma(
            collection_name=self.collection_name,
            embedding_function=self.embeddings,
            collection_metadata={"hnsw:space": "cosine"}
        )

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
        if not self.my_collection:
            raise Exception("Collection not initialized. Call connect() first.")
        self.my_collection.delete(ids=[vector_id])
    