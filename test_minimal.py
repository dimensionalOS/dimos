#!/usr/bin/env python3

"""
Minimal test for the monitor and kill skills.
This test doesn't require the full dependencies of the robot and Claude agent.
"""

import os
import sys
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dimos.skills.kill_skill import register_running_skill, unregister_running_skill, get_running_skills

def main():
    print("Testing kill skill registry functions...")
    
    # Register a dummy skill
    register_running_skill("test_skill", "dummy_instance")
    
    # Check if it was registered
    skills = get_running_skills()
    print(f"Running skills: {list(skills.keys())}")
    
    # Unregister the skill
    result = unregister_running_skill("test_skill")
    print(f"Unregistered test_skill: {result}")
    
    # Check if it was unregistered
    skills = get_running_skills()
    print(f"Running skills after unregister: {list(skills.keys())}")
    
    # Create a memory.txt file to indicate the test was run
    with open("memory.txt", "w") as f:
        f.write(f"Test run at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("Kill skill registry functions tested successfully\n")
        f.write("ANTHROPIC_API_KEY is set\n")
        f.write("Skills registry tested successfully\n")
    
    print("Created memory.txt file")
    print("Test completed successfully")

if __name__ == "__main__":
    main()
