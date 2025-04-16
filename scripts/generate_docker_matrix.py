#!/usr/bin/env python3
"""
Docker image build matrix generator for GitHub Actions.
- Finds all Dockerfiles in the repository
- Generates appropriate image names and tags based on paths
- Creates a JSON matrix for GitHub Actions workflow
"""

import os
import re
import sys
import json
import subprocess
import argparse
from pathlib import Path
from typing import Dict, List, Set, Tuple


# ANSI colors for terminal output
class Colors:
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"


def get_repo_root() -> Path:
    """Get the git repository root directory."""
    try:
        root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], universal_newlines=True
        ).strip()
        return Path(root)
    except subprocess.CalledProcessError:
        print(f"{Colors.RED}Not a git repository.{Colors.ENDC}")
        return Path(os.getcwd())


def find_dockerfiles(root_dir: Path, exclude_patterns: List[str] = None) -> List[Path]:
    """Find all Dockerfiles in the repository, excluding patterns."""
    if exclude_patterns is None:
        exclude_patterns = ["dimos/robot/unitree/external"]

    dockerfiles = []
    for dockerfile in root_dir.glob("**/Dockerfile"):
        # Convert to string for pattern matching
        dockerfile_str = str(dockerfile)
        if not any(pattern in dockerfile_str for pattern in exclude_patterns):
            dockerfiles.append(dockerfile)

    return sorted(dockerfiles)


def generate_image_name(dockerfile: Path, root_dir: Path, registry: str) -> str:
    """Generate a Docker image name from its path."""
    # Get path relative to docker directory
    rel_path = dockerfile.relative_to(root_dir)

    # Extract components after 'docker/' directory
    if "docker" in str(rel_path).split(os.sep):
        components = str(rel_path).split(os.sep)[
            1:-1
        ]  # Remove 'docker/' and 'Dockerfile'
    else:
        components = str(rel_path).split(os.sep)[:-1]  # Just remove 'Dockerfile'

    # Create name with dashes
    image_name = "-".join(components)

    # Use convention: dimensionalos/{image_name}
    return f"{registry}/{image_name}"


def generate_matrix_json(
    dockerfiles: List[Path],
    root_dir: Path,
    registry: str,
    include_registry: bool = True,
) -> str:
    """Generate GitHub Actions matrix JSON for Docker builds."""
    matrix = []

    for df in dockerfiles:
        # Get path relative to repo root for GitHub Actions
        rel_path = str(df.relative_to(root_dir))

        # Generate the full image name with registry
        full_image_name = generate_image_name(df, root_dir, registry)

        # Extract name part from image name (without registry)
        name = full_image_name.replace(f"{registry}/", "")

        # Use full image name or just the name part based on flag
        image_name = full_image_name if include_registry else name

        # Get the context directory (parent directory of the Dockerfile)
        context_dir = str(df.parent.relative_to(root_dir))

        matrix.append(
            {
                "dockerfile": rel_path,
                "context": context_dir,
                "name": name,
                "image": image_name,
            }
        )

    return json.dumps(matrix, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Docker build matrix generator for GitHub Actions"
    )
    parser.add_argument(
        "--registry", default="dimensionalos", help="Docker registry name"
    )
    parser.add_argument(
        "--exclude-registry",
        action="store_true",
        help="Exclude registry prefix from image names in output",
    )
    parser.add_argument(
        "--output",
        choices=["stdout", "file"],
        default="stdout",
        help="Output to stdout or file",
    )
    parser.add_argument(
        "--file",
        default="docker-matrix.json",
        help="Output file name (if --output=file)",
    )
    args = parser.parse_args()

    root_dir = get_repo_root()
    # print(f"{Colors.BLUE}Finding Dockerfiles in {root_dir}...{Colors.ENDC}")

    dockerfiles = find_dockerfiles(root_dir)
    # print(f"{Colors.GREEN}Found {len(dockerfiles)} Dockerfiles{Colors.ENDC}")

    # Generate the matrix JSON
    include_registry = not args.exclude_registry
    matrix = []
    
    for df in dockerfiles:
        # Get path relative to repo root for GitHub Actions
        rel_path = str(df.relative_to(root_dir))
        
        # Generate the full image name with registry
        full_image_name = generate_image_name(df, root_dir, args.registry)
        
        # Extract name part from image name (without registry)
        name = full_image_name.replace(f"{args.registry}/", "")
        
        # Use full image name or just the name part based on flag
        image_name = full_image_name if include_registry else name
        
        # Get the context directory (parent directory of the Dockerfile)
        context_dir = str(df.parent.relative_to(root_dir))
        
        matrix.append({
            "dockerfile": rel_path,
            "context": context_dir,
            "name": name,
            "image": image_name,
        })
    
    matrix_json = json.dumps(matrix)
    
    # Output based on user preference
    if args.output == "file":
        output_path = Path(args.file)
        with open(output_path, "w") as f:
            f.write(matrix_json)
        print(f"{Colors.GREEN}Matrix written to {output_path}{Colors.ENDC}")
    else:
        # Always use GitHub Actions output format without the matrix= prefix
        # The workflow will add this to GITHUB_OUTPUT with the correct format
        print(matrix_json)


if __name__ == "__main__":
    main()
