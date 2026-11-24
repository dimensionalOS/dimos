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

Expected result: the full suite passes without skips. Record the result against the tested SHA.

For iteration, `./bin/pytest-slow` includes the default, browser and native tests but excludes `self_hosted_large`. Plain `uv run pytest` also excludes browser and native end-to-end tests. Neither reduced run completes this release check.

Review the [CI workflow](../../.github/workflows/ci.yml), [installation workflow](../../.github/workflows/install.yml) and [e2e workflow](../../.github/workflows/e2e.yml) results for the same release-branch SHA, including the web and native build jobs outside pytest. Dispatch these workflows on the release branch when needed: a push to `release/*` does not automatically start the main CI workflow.

The e2e workflow runs the full `self_hosted_large` and `web_browser` suites. Review all three jobs: `self-hosted-large-tests`, `browser-tests-linux` and `browser-tests-macos`. A passing local `./bin/pytest-all` run does not replace these results, including the macOS browser coverage.

The daily e2e schedule runs on `main`. If there is no successful run for `RELEASE_COMMIT`, use **Run workflow** (`workflow_dispatch`) on the release branch in GitHub Actions. Verify that the run's commit matches `RELEASE_COMMIT`; a scheduled run for another SHA is not sufficient. Record the run URL, SHA and each job's conclusion in the release test record. Do not mark section 1.1 passed while an e2e job is failed, cancelled, skipped, pending or missing. All three jobs must complete successfully for the candidate being signed off. If the release branch advances, repeat the checks for the new candidate.

## 1.2 Type checking

```bash
uv run mypy dimos
```

Expected result: mypy finishes without errors.
