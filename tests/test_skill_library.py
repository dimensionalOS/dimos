import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----

from dimos.robot.skill_library import SkillLibrary

library = SkillLibrary()

# Describing a skill
print(library.describe_skill("MoveX"))

# Calling a skill
print(library.call_skill("MoveX", distance=10))
print(library.call_skill("GripArm"))

# Describing all available skills
print(library.describe_all_skills())

# Trying to describe or call an unknown skill
print(library.describe_skill("Fly"))
print(library.call_skill("Fly"))
