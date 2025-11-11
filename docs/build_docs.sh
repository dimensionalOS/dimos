#!/bin/bash
# Build MkDocs documentation

set -e

echo "Building DimOS documentation..."

# Ensure docs dependencies are installed
uv pip install -e .[docs]

# Build the documentation
uv run mkdocs build -f mkdocs.yml

echo "Documentation built successfully in site/ directory"
