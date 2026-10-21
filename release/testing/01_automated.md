# 1. Automated checks

Use the release branch and commit recorded in the [shared setup](README.md#before-testing), with the dependencies, credentials and test data required by the full suite. See the [testing guide](../../docs/development/testing.md) for setup and test selection details.

## 1.1 Tests

Run on a host that can execute the high-memory simulation tests as well as the default suite. The sync needs CycloneDDS at `/opt/cyclonedds`. See the [Ubuntu apt + symlink shim](../../docs/usage/transports/dds.md#alternative-ubuntu-apt--symlink-shim).

```bash
test "$(git rev-parse HEAD)" = "${RELEASE_COMMIT:?Set RELEASE_COMMIT from the release test record}"
CYCLONEDDS_HOME=/opt/cyclonedds uv sync --locked --all-extras --all-groups
uv run playwright install chromium firefox
git lfs pull --include="misc/DimSim/**"
bin/dimsim-prepare
# Builds native test dependencies and fetches LFS test data before running every pytest marker.
./bin/pytest-all
```

Expected result: the full suite passes without skips, as required by the [release procedure](../../docs/development/releasing.md#2-creating-the-release). Record the result against the tested SHA.

For iteration, `./bin/pytest-slow` includes the default, browser and native tests but excludes `self_hosted_large`. Plain `uv run pytest` also excludes browser and native end-to-end tests. Neither reduced run completes this release check.

Review the [CI workflow](../../.github/workflows/ci.yml) and [installation workflow](../../.github/workflows/install.yml) results for the same release-branch SHA, including the web and native build jobs outside pytest. Dispatch these workflows on the release branch when needed: a push to `release/*` does not automatically start the main CI workflow.

## 1.2 Type checking

```bash
uv run mypy dimos
```

Expected result: mypy finishes without errors.
