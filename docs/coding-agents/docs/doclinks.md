(documentation-links)=

# Links and Cross-References

Use standard Markdown links for pages and sections. MyST resolves them as
Sphinx cross-references, and the strict build reports missing targets.

| Target | Syntax |
| --- | --- |
| Another page | `[Configuration](../../usage/configuration.md)` |
| Named section | `[Configurable modules](../../usage/configuration.md#configurable-modules)` |
| Python class | ``[`In`][In]`` |
| Python function | ``[`autoconnect()`][autoconnect]`` |
| External site | A standard Markdown link, such as `[Sphinx](https://www.sphinx-doc.org/)` |

Define each Python target once at the bottom of the page:

```md
[In]: #dimos.core.stream.In
[autoconnect]: #dimos.core.coordination.blueprints.autoconnect
```

Use GitHub links for files that are not part of the Sphinx documentation,
such as a package-specific `README.md`. Prefer Markdown reference links for
Python symbols. They render on GitHub, and Sphinx resolves their targets
against the generated API documentation.
