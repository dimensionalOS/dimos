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
from typing import Dict, List, Set, Tuple, Optional


def get_repo_root() -> Path:
    """Get the git repository root directory."""
    try:
        root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], universal_newlines=True
        ).strip()
        return Path(root)
    except subprocess.CalledProcessError:
        print("Not a git repository.")
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


def get_base_image(dockerfile: Path, registry: str) -> Optional[str]:
    """Extract the base image from a Dockerfile's FROM statement.

    Returns:
        The base image name if it's from our registry, otherwise None.
    """
    try:
        with open(dockerfile, "r") as f:
            content = f.read()

        # Extract the FROM statement
        from_match = re.search(
            r"^\s*FROM\s+([^\s:]+)(?::[\w\.\-]+)?(?:\s+AS\s+\w+)?",
            content,
            re.MULTILINE,
        )

        if not from_match:
            return None

        base_image = from_match.group(1)

        # Only return if it's from our registry
        if base_image.startswith(f"{registry}/"):
            return base_image

        return None

    except Exception as e:
        print(f"Error extracting base image for {dockerfile}: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Docker build matrix generator for GitHub Actions"
    )
    parser.add_argument(
        "--registry", default="dimensionalos", help="Docker registry name"
    )
    args = parser.parse_args()

    root_dir = get_repo_root()
    dockerfiles = find_dockerfiles(root_dir)

    # First pass: Create entries with direct dependencies
    entries = {}
    image_to_entry = {}

    for df in dockerfiles:
        # Get path relative to repo root for GitHub Actions
        rel_path = str(df.relative_to(root_dir))

        # Generate the full image name with registry
        image_name = generate_image_name(df, root_dir, args.registry)

        # Extract name part from image name (without registry)
        name = image_name.replace(f"{args.registry}/", "")

        # Identify direct dependency
        base_image = get_base_image(df, args.registry)

        entry = {
            "dockerfile": rel_path,
            "name": name,
            "image": image_name,
            "dependency": base_image,
            "level": 0,  # Default level, will be calculated later
        }

        entries[image_name] = entry
        image_to_entry[image_name] = entry

    # Second pass: Calculate levels
    # Set level 0 for images with no dependencies on our registry
    for image_name, entry in entries.items():
        if entry["dependency"] is None:
            entry["level"] = 0

    # Iteratively resolve levels for the rest
    changes_made = True
    while changes_made:
        changes_made = False

        for image_name, entry in entries.items():
            dependency = entry["dependency"]
            if dependency and dependency in entries:
                # If this entry depends on another image with a known level
                if entries[dependency]["level"] >= 0:
                    new_level = entries[dependency]["level"] + 1
                    if entry["level"] != new_level:
                        entry["level"] = new_level
                        changes_made = True

    # Clean up the entries before output (remove the dependency field)
    for entry in entries.values():
        del entry["dependency"]

    # Convert to list for output
    matrix = list(entries.values())

    # Sort by level to ensure build order is correct
    matrix.sort(key=lambda x: x["level"])

    print(json.dumps(matrix))


if __name__ == "__main__":
    main()
