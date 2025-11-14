import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----

import chromadb
from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise Exception("OpenAI key not specified.")

collection_name = "my_collection"

embeddings = OpenAIEmbeddings(
    model="text-embedding-3-large",
    dimensions=1024,
    api_key=OPENAI_API_KEY,
)

db_connection = Chroma(
    collection_name=collection_name,
    embedding_function=embeddings,
)

def add_vector(vector_id, vector_data):
    """Add a vector to the ChromaDB collection."""
    if not db_connection:
        raise Exception("Collection not initialized. Call connect() first.")
    db_connection.add_texts(
        ids=[vector_id],
        texts=[vector_data],
        metadatas=[{"name": vector_id}],
    )

add_vector("id0", "Food")
add_vector("id1", "Cat")
add_vector("id2", "Mouse")
add_vector("id3", "Bike")
add_vector("id4", "Dog")
add_vector("id5", "Tricycle")
add_vector("id6", "Car")
add_vector("id7", "Horse")
add_vector("id8", "Vehicle")
add_vector("id6", "Red")
add_vector("id7", "Orange")
add_vector("id8", "Yellow")


def get_vector(vector_id):
    """Retrieve a vector from the ChromaDB by its identifier."""
    result = db_connection.get(include=['embeddings'], ids=[vector_id])
    return result

print(get_vector("id1"))
# print(get_vector("id3"))
# print(get_vector("id0"))
# print(get_vector("id2"))

def query(query_texts, n_results=2):
    """Query the collection with a specific text and return up to n results."""
    if not db_connection:
        raise Exception("Collection not initialized. Call connect() first.")
    return db_connection.similarity_search(
        query=query_texts,
        k=n_results
    )

results = query("Colors")
print(results)
