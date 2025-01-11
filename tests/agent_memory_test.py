import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# -----

from dotenv import load_dotenv
import os

from dimos.agents.memory.chroma_impl import AgentSemanticMemory 
print("Hi.")

agent_memory = AgentSemanticMemory()
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
print("Done querying agent memory.")
