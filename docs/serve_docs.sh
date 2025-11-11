#!/bin/bash
# Serve MkDocs documentation locally with hot reload

set -e

echo "Starting MkDocs development server..."

# Ensure docs dependencies are installed
uv pip install -e .[docs]

# Serve the documentation with hot reload
echo "Documentation will be available at http://127.0.0.1:8000/"
uv run mkdocs serve -f mkdocs.yml
