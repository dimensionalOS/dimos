# Apple Silicon macOS

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

## Transport note for macOS

LCM over UDP can be unreliable on macOS for large or high-rate replay workloads. dimOS defaults the global stream transport to **Zenoh** everywhere, so you never need `--transport=zenoh`. Use `--transport=lcm` if you need to force the legacy multicast path.

See the [Zenoh quickstart](/docs/usage/transports/index.md#zenoh-quickstart) for what the localhost-pinned default reaches and how to point it at a robot or the LAN.

```sh skip
dimos --dtop --replay --replay-db=go2_bigoffice run unitree-go2
```

If you are developing on the repository, prefer syncing the full environment with the checked-in lockfile:

```sh skip
uv sync --extra all --frozen
```
