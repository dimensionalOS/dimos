import lcm
import sys
import os

# Redirect stderr to stdout
sys.stderr = sys.stdout

try:
    print("Attempting to create LCM object...")
    l = lcm.LCM()
    print("Successfully created LCM object")
except Exception as e:
    print(f"Error creating LCM object: {e}")
