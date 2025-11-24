import tests.test_header
import os

# -----

from dotenv import load_dotenv
import os

load_dotenv()

from dimos.agents.memory.chroma_impl import OpenAISemanticMemory

agent_memory = OpenAISemanticMemory()
print("Initialization done.")

agent_memory.add_vector("id0", "Food")
agent_memory.add_vector("id1", "Cat")
agent_memory.add_vector("id2", "Mouse")
agent_memory.add_vector("id3", "Bike")
agent_memory.add_vector("id4", "Dog")
agent_memory.add_vector("id5", "Tricycle")
agent_memory.add_vector("id6", "Car")
agent_memory.add_vector("id7", "Horse")
agent_memory.add_vector("id8", "Vehicle")
agent_memory.add_vector("id6", "Red")
agent_memory.add_vector("id7", "Orange")
agent_memory.add_vector("id8", "Yellow")
print("Adding vectors done.")

print(agent_memory.get_vector("id1"))
print("Done retrieving sample vector.")

results = agent_memory.query("Colors")
print(results)
print("Done querying agent memory (basic).")

results = agent_memory.query("Colors", similarity_threshold=0.2)
print(results)
print("Done querying agent memory (similarity_threshold=0.2).")

results = agent_memory.query("Colors", n_results=2)
print(results)
print("Done querying agent memory (n_results=2).")

results = agent_memory.query("Colors", n_results=19, similarity_threshold=0.45)
print(results)
print("Done querying agent memory (n_results=19, similarity_threshold=0.45).")