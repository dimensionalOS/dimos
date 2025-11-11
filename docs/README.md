# DimOS Documentation

This directory contains the MkDocs documentation for DimOS.

## Structure

```
docs/
├── SUMMARY.md              # Navigation definition (literate-nav)
├── index.md                # Homepage
├── build_docs.sh           # Build static site
├── serve_docs.sh           # Local dev server with hot reload
├── quickstart/             # Getting started guides
│   ├── index.md
│   ├── installation.md
│   └── basic-usage.md
├── tutorials/              # Step-by-step tutorials
│   ├── index.md
│   ├── simple-agent.md
│   ├── agent-chaining.md
│   └── custom-skills.md
└── api/                    # API reference (auto-generated)
    ├── index.md
    └── agents.md
```

## Setup

### 1. Create and activate virtual environment

```bash
uv venv
source .venv/bin/activate
```

### 2. Install documentation dependencies

```bash
uv pip install -e '.[docs]'
```

Note: The quotes around `'.[docs]'` are required in zsh to prevent glob expansion.

## Building Documentation

### Local Development Server

Start a local server with hot reload:

```bash
./docs/serve_docs.sh
# or
mkdocs serve
```

Then open http://127.0.0.1:8000/ in your browser.

### Build Static Site

Build the static documentation site:

```bash
./docs/build_docs.sh
# or
mkdocs build
```

Output will be in the `site/` directory.

## Documentation Architecture

### Navigation

- Uses **Material for MkDocs** theme with navigation tabs
- Top-level sections (Quickstart, Tutorials, API Reference) appear as horizontal tabs
- Navigation structure defined in `SUMMARY.md` using literate-nav plugin
- Flat hierarchy (max 2-3 levels) for easy scanning

### API Documentation

- Uses **mkdocstrings** to auto-generate API docs from Python docstrings
- Uses `:::` syntax to include module documentation
- Example: `::: dimos.agents` extracts all docstrings from the agents module


## Design Principles

1. **Flat hierarchy** - Maximum 2-3 levels of nesting
2. **Navigation tabs** - Top-level sections in horizontal tab bar
3. **Card-based landing pages** - Visual overview of sections
4. **Auto-generated API docs** - Maintained through docstrings
5. **Simple and scannable** - Easy to find information quickly

## Deployment

To deploy to GitHub Pages:

```bash
mkdocs gh-deploy
```

Or set up GitHub Actions for automatic deployment on push to main.
