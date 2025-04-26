
#
#
#

"""
Script to run pytest tests for the DIMOS project.

This script provides a convenient way to run the pytest tests with various options.
"""

import os
import sys
import argparse
import subprocess


def main():
    """Run pytest tests with specified options."""
    parser = argparse.ArgumentParser(description="Run pytest tests for DIMOS")
    
    parser.add_argument(
        "--module", "-m",
        help="Run tests for a specific module (e.g., skills, agents, robot)",
        default=None
    )
    
    parser.add_argument(
        "--hardware",
        help="Include tests that require physical hardware",
        action="store_true"
    )
    
    parser.add_argument(
        "--verbose", "-v",
        help="Run tests in verbose mode",
        action="store_true"
    )
    
    parser.add_argument(
        "--coverage",
        help="Generate coverage report",
        action="store_true"
    )
    
    args = parser.parse_args()
    
    cmd = ["python", "-m", "pytest"]
    
    if args.verbose:
        cmd.append("-v")
    
    if not args.hardware:
        cmd.append("-m")
        cmd.append("not hardware")
    
    if args.coverage:
        cmd.append("--cov=dimos")
        cmd.append("--cov-report=term")
        cmd.append("--cov-report=html")
    
    if args.module:
        cmd.append(f"tests/{args.module}")
    
    print(f"Running: {' '.join(cmd)}")
    
    result = subprocess.run(cmd)
    
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
