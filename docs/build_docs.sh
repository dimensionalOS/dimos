#!/bin/bash
# Build MkDocs documentation
# The post-build hook (docs/hooks.py) automatically generates llms-ctx files

set -e

echo "Building DimOS documentation..."

# Install docs dependencies if needed (avoids full project resolution issues)
# In devcontainer, these should already be installed
if ! command -v mkdocs &> /dev/null; then
    echo "Installing docs dependencies..."
    uv pip install mkdocs mkdocs-material "mkdocstrings[python]" mkdocs-llmstxt llm-ctx --system
fi

# Build the documentation (hooks run automatically)
mkdocs build -f mkdocs.yml

echo ""
echo "Documentation built successfully in site/ directory"
echo "Generated files:"
echo "  - site/llms.txt (link index)"
echo "  - site/llms-full.txt (complete content)"
echo "  - site/llms-ctx.txt (Claude-optimized)"
echo "  - site/llms-ctx-full.txt (Claude-optimized with optional sections)"
