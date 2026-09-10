# Ubuntu

For SDK applications, follow the [dimup installation guide](/docs/installation/installer.md).

## Developing on dimOS

```sh skip
# this allows getting large files on-demand (and not pulling all immediately)
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

# Install all dependency groups (tests, lint, …) so mypy + pytest are
# both available. For self-hosted tests, see docs/development/testing.md.
uv sync --all-groups

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest --numprocesses=auto dimos
```
