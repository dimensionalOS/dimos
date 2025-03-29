import tests.test_header
import os
import sys

# -----

# Now try to import
from dimos.environment.colmap_environment import COLMAPEnvironment

env = COLMAPEnvironment()
env.initialize_from_video("data/IMG_1525.MOV", "data/frames")
