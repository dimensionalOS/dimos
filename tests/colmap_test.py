import sys
import os

# Add the parent directory of 'demos' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----

# Now try to import
from dimos.environment.colmap_environment import COLMAPEnvironment

env = COLMAPEnvironment()
env.initialize_from_video("data/IMG_1525.MOV", "data/frames")
