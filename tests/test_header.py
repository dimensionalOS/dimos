"""Test utilities for identifying caller information and path setup.

This module provides functionality to determine which file called the current
script and sets up the Python path to include the parent directory, allowing
tests to import from the main application.
"""

import sys
import os
import inspect

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def get_caller_info():
    """Identify the filename of the caller in the stack.
    
    Examines the call stack to find the first non-internal file that called
    this module. Skips the current file and Python internal files.
    
    Returns:
        str: The basename of the caller's filename, or "unknown" if not found.
    """
    current_file = os.path.abspath(__file__)
    
    # Look through the call stack to find the first file that's not this one
    for frame in inspect.stack()[1:]:
        filename = os.path.abspath(frame.filename)
        # Skip this file and Python internals
        if filename != current_file and "<frozen" not in filename and "__pycache__" not in filename:
            return os.path.basename(filename)
    
    # If we can't find a caller, print the stack for debugging
    print("Call stack:")
    for i, frame in enumerate(inspect.stack()):
        print(f"  {i}: {frame.filename}")
    
    return "unknown"

caller_filename = get_caller_info()
print(f"Hi from: \033[34m{caller_filename}\033[0m")
print(f"Current working directory: \033[34m{os.getcwd()}\033[0m\n")
