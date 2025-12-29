When writing or editing markdown documentation, use `doclinks` tool to resolve file references.

## Link Syntax

### Code file references
Use backticks around the filename:
```markdown
See [`service/spec.py`]() for the implementation.
```

After running doclinks, becomes:
```markdown
See [`service/spec.py`](/dimos/protocol/service/spec.py) for the implementation.
```

### Symbol auto-linking
Mention a symbol on the same line to auto-link to its line number:
```markdown
The `Configurable` class is defined in [`service/spec.py`]().
```

Becomes:
```markdown
The `Configurable` class is defined in [`service/spec.py`](/dimos/protocol/service/spec.py#L22).
```

### Doc-to-doc references
Use `.md` as the link target:
```markdown
See [Configuration](.md) for more details.
```

Becomes:
```markdown
See [Configuration](/docs/concepts/configuration.md) for more details.
```

## Running doclinks

After editing documentation, run:
```bash
doclinks <file.md>
```

Or for all docs:
```bash
doclinks docs/
```

## Options
- `--dry-run` - Preview changes without writing
- `--link-mode github --github-url <url>` - Generate GitHub URLs instead of repo-relative paths
- `--link-mode relative` - Generate paths relative to the doc file

## File matching

- Code files: Match by filename suffix (e.g., `service/spec.py` matches `/dimos/protocol/service/spec.py`)
- Doc files: Match by stem, case-insensitive (e.g., `Configuration` matches `configuration.md` or `configuration/index.md`)

## Error handling

- **Ambiguous match**: If multiple files match (e.g., `spec.py` exists in multiple dirs), use a longer path like `service/spec.py`
- **No match**: Check the filename spelling and ensure the file exists in the codebase
