(coding-agent-documentation)=

# Writing DimOS Documentation

DimOS documentation is authored as MyST Markdown in `docs/` and built with
Sphinx. Add user-facing guides under `docs/usage` or
`docs/capabilities` and contributor-only material under
`docs/development`. Include each new page in the nearest `toctree`.

Use checked cross-references so renamed pages and Python symbols fail the
strict build:

```md
See [Configuration](../../usage/configuration.md) for configuration precedence.
Streams use [`dimos.core.stream.In`][In] and
[`dimos.core.stream.Out`][Out].
Jump to [Configurable modules](../../usage/configuration.md#configurable-modules).
```

Prefer checked-in examples with `literalinclude` over copied snippets.
See [Code Examples](codeblocks.md) for code examples and [Links and Cross-References](doclinks.md) for links.

Before submitting a documentation change, run:

```bash
uv sync --only-group docs
uv run make -C docs html
uv run make -C docs spelling
```

Both commands treat warnings as errors in CI.

```{toctree}
:hidden: true
:maxdepth: 1

codeblocks
doclinks
```

[In]: #dimos.core.stream.In
[Out]: #dimos.core.stream.Out
