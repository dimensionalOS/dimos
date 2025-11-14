# DimOS Documentation

This directory contains the MkDocs documentation for DimOS.

## Installation

If you are using the devcontainer, you don't need to install anything else—the dependencies for the docs site are already included.

## Building Documentation

### Local Development Server

Start a local server with hot reload:

```bash
mkdocs serve
```

Then open <http://127.0.0.1:8000/> in your browser.

### Build Static Site

Build the static documentation site:

```bash
mkdocs build
```

Output (which includes the various `llm.tx`es) will be in the `site/` directory.

## Documentation Architecture

### Navigation

- Uses **Material for MkDocs** theme with navigation tabs
- Top-level sections (Quickstart, Tutorials, API Reference) appear as horizontal tabs
- Navigation structure defined in `SUMMARY.md` using literate-nav plugin
- Flat hierarchy (max 2-3 levels) for easy scanning

### API Documentation

- Uses **mkdocstrings** to auto-generate API docs from Python docstrings
- Uses `:::` syntax to include module documentation

## Design Principles

1. **Flat hierarchy** - Maximum 2-3 levels of nesting
2. **Navigation tabs** - Top-level sections in horizontal tab bar
3. **Card-based landing pages** - Visual overview of sections
4. **Auto-generated API docs** - Maintained through docstrings
5. **Simple and scannable** - Easy to find information quickly

<!-- TODO: Add deployment details later -->