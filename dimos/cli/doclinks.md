# doclinks

A Markdown link resolver that automatically fills in correct file paths for code references in documentation.

## What it does

When writing docs, you can use placeholder links like:

<!-- doclinks-ignore-start -->
```markdown
See [`service/spec.py`]() for the implementation.
```
<!-- doclinks-ignore-end -->

Running `doclinks` resolves these to actual paths:

<!-- doclinks-ignore-start -->
```markdown
See [`service/spec.py`](/dimos/protocol/service/spec.py) for the implementation.
```
<!-- doclinks-ignore-end -->

## Features

<!-- doclinks-ignore-start -->
- **Code file links**: `[`filename.py`]()` resolves to the file's path
- **Symbol line linking**: If another backticked term appears on the same line, it finds that symbol in the file and adds `#L<line>`:
  ```markdown
  See `Configurable` in [`config.py`]()
  → [`config.py`](/path/config.py#L42)
  ```
- **Doc-to-doc links**: `[Modules](.md)` resolves to `modules.md` or `modules/index.md`
<!-- doclinks-ignore-end -->
- **Multiple link modes**: absolute, relative, or GitHub URLs
- **Watch mode**: Automatically re-process on file changes
- **Ignore regions**: Skip sections with `<!-- doclinks-ignore-start/end -->` comments

## Usage

```bash
# Process a single file
doclinks docs/guide.md

# Process a directory recursively
doclinks docs/

# Relative links (from doc location)
doclinks --link-mode relative docs/

# GitHub links
doclinks --link-mode github \
  --github-url https://github.com/org/repo docs/

# Dry run (preview changes)
doclinks --dry-run docs/

# CI check (exit 1 if changes needed)
doclinks --check docs/

# Watch mode (auto-update on changes)
doclinks --watch docs/
```

## Options

| Option             | Description                                     |
|--------------------|-------------------------------------------------|
| `--root PATH`      | Repository root (default: auto-detect git root) |
| `--link-mode MODE` | `absolute` (default), `relative`, or `github`   |
| `--github-url URL` | Base GitHub URL (required for github mode)      |
| `--github-ref REF` | Branch/ref for GitHub links (default: `main`)   |
| `--dry-run`        | Show changes without modifying files            |
| `--check`          | Exit with error if changes needed (for CI)      |
| `--watch`          | Watch for changes and re-process                |

## Link patterns

<!-- doclinks-ignore-start -->
| Pattern              | Description                                    |
|----------------------|------------------------------------------------|
| `[`file.py`]()`      | Code file reference (empty or any link)        |
| `[`path/file.py`]()` | Code file with partial path for disambiguation |
| `[`file.py`](#L42)`  | Preserves existing line fragments              |
| `[Doc Name](.md)`    | Doc-to-doc link (resolves by name)             |
<!-- doclinks-ignore-end -->

## How resolution works

The tool builds an index of all files in the repo. For `/dimos/protocol/service/spec.py`, it creates lookup entries for:

- `spec.py`
- `service/spec.py`
- `protocol/service/spec.py`
- `dimos/protocol/service/spec.py`

Use longer paths when multiple files share the same name.
