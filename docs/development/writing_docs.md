(writing-docs)=

# Writing Docs

DimOS uses Sphinx with MyST Markdown sources in `docs/`. The generated API
reference remains in `docs/api.rst`, where Sphinx's autodoc directives are
most concise.

Place user-facing guides in `docs/usage` or `docs/capabilities` and
contributor-only guides in `docs/development`. Add every new page to the
nearest `toctree` so readers can discover it.

Use standard Markdown links for documentation pages and sections. Use
Markdown reference links for Python symbols. Both forms render cleanly on
GitHub and remain checked by Sphinx. Prefer `literalinclude` for source
examples, and keep images beside the relevant section in an `assets`
directory. See
[Writing DimOS Documentation](../coding-agents/docs/index.md) for the complete authoring conventions.

Run the same strict checks as CI:

```bash
uv sync --only-group docs
uv run make -C docs html
uv run make -C docs spelling
```

The HTML and spelling targets fail on warnings.
