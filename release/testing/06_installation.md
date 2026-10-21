# 6. Installation

Run these checks on fresh machines without an earlier DimOS setup. The Ubuntu check uses a Ubuntu 24.04 VirtualBox VM. Run its commands inside the VM.

Copy `RELEASE_VERSION`, `RELEASE_BRANCH` and the full `RELEASE_COMMIT` SHA from the [release test record](README.md#before-testing) into each terminal used below. The installer script is fetched from that SHA and receives the release branch explicitly. After installation, verify that the checkout still matches the recorded SHA; a mismatch means the branch advanced and the check must be repeated for the new candidate.

## 6.1 New Ubuntu install

* Download a [Ubuntu 24.04](https://releases.ubuntu.com/24.04/ubuntu-24.04.4-desktop-amd64.iso) image and install it in VirtualBox.
* Follow the guided installation flow from the [Ubuntu guide](../../docs/installation/ubuntu.md), using the release-specific command below. Accept the defaults, including the replay smoke test.

```bash
sudo apt-get update && sudo apt-get install -y curl
curl -fsSL "https://raw.githubusercontent.com/dimensionalOS/dimos/${RELEASE_COMMIT:?}/scripts/install.sh" \
  | bash -s -- --mode dev --branch "${RELEASE_BRANCH:?}" --no-nix --project-dir ./dimos
# In a new terminal, so uv is on PATH:
cd dimos && source .venv/bin/activate
git branch --show-current
git rev-parse HEAD
test "$(git rev-parse HEAD)" = "${RELEASE_COMMIT:?}"
```

Expected result: the installer prints "installation verified" and "smoke test passed", replay appears in Rerun, and the branch and commit match the release test record.

## 6.2 New macOS install

Use an Apple Silicon Mac on macOS 14 or newer, see the [macOS guide](../../docs/installation/osx.md). Accept the installer defaults, including the replay smoke test.

```bash
curl -fsSL "https://raw.githubusercontent.com/dimensionalOS/dimos/${RELEASE_COMMIT:?}/scripts/install.sh" \
  | bash -s -- --mode dev --branch "${RELEASE_BRANCH:?}" --project-dir ./dimos
# In a new terminal, so uv is on PATH:
cd dimos && source .venv/bin/activate
git branch --show-current
git rev-parse HEAD
test "$(git rev-parse HEAD)" = "${RELEASE_COMMIT:?}"
uv run dimos --simulation run unitree-go2
```

Expected result: the branch and commit match the release test record, the installer prints "installation verified" and "smoke test passed", and replay appears in Rerun. The separate simulation run opens MuJoCo and Rerun with the robot in them.

## 6.3 Native modules built with Nix

Install Nix with flakes enabled and add the Cachix substituter from [Native modules](../../docs/usage/native_modules.md).

```bash
bin/native-modules --dry-run
bin/native-modules --build
uv run dimos --build-native run go2-sim-nav
```

Expected result: the dry run ends with "Would build: nothing" when the cache has every module. The build prints a DONE line per module and no FAIL. The forced rebuild logs "Build command completed" for each native module before the simulated Go2 appears in Rerun.

## 6.4 Published release

Run this after the release workflow has finished, from the repository root. Use the version and tested release-branch SHA from the release test record.

```bash
gpg --import docs/release-signing-key.asc
git fetch origin --tags
git tag -v "v${RELEASE_VERSION:?}"
test "$(git rev-parse "v$RELEASE_VERSION^{commit}")" = "${RELEASE_COMMIT:?}"
# Install the published wheel in an empty directory outside the checkout.
cd "$(mktemp -d)" && uv venv --python 3.12 && source .venv/bin/activate
uv pip install --torch-backend cpu "dimos[base,unitree,sim]==$RELEASE_VERSION"
uv run dimos --replay run unitree-go2
```

* https://github.com/dimensionalOS/dimos/releases shows the release version with the sdist and wheels attached. GitHub marks the tag "Unverified", which is expected.
* https://pypi.org/project/dimos/ shows the release version.

Expected result: `git tag -v` reports a good signature from `dimos-release-bot`, the tag points to the tested release-branch commit, the wheel installs, and the replay shows in Rerun without tracebacks.
