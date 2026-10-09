# Canonical source-only native package

This Rust probe is the small reference for the [package API](/docs/usage/packages.md).
`tool.dimos` supplies source packaging and blueprint metadata; no Cargo/CMake runs
while building or installing the Python distribution. The build integration is not
published: first build `packages/dimos-build-config` and provide its wheel through
`PIP_FIND_LINKS` or `UV_FIND_LINKS`. The host must contain `source_package` support.

This project ships two Rust binaries as source in one ordinary Python wheel.
Installing it or listing blueprints does not invoke Cargo, rustc or CMake.
Only the selected blueprint's binary is built, just before its module starts.
The [legacy prebuilt fixture](../native/README.md) is retained only for runtime compatibility tests.

Use a dimOS host containing `NativeModuleConfig.source_package`. This API is not
in the currently published 0.0.14 host; the example's version constraint alone
cannot distinguish an unreleased checkout from that release. For development,
activate the matching host environment, then from this directory:

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps .
export DIMOS_PACKAGE_REPORT=/tmp/dimos-lazy-native-report.txt
dimos list
dimos --transport zenoh --viewer none --n-workers 1 --no-serve-coordinator-rpc run dimos-lazy-native.probe
```

Installation needs Python 3.10–3.12, scikit-build-core and the local dimos-build-config wheel.
Running this POSIX example additionally needs Cargo and rustc; there are no
third-party crates or runtime SDK downloads. If prompted to apply system
configuration changes, answer `n`. Wait for `Blueprint started`, then Ctrl-C.
The report contains `ready <PID> hello from a lazily built native package`, followed
by `stopped`. No hardware or typed transport is exercised.

`probe` builds only `package_probe`. `dimos-lazy-native.other` selects
`package_other` instead. The explicit `--bin` argument selects a Cargo target;
DimOS does not infer targets from blueprint names. Installing from a wheel or
source uses scikit-build-core to copy the complete `native/` tree, including the lockfile,
into the wheel. The backend never invokes Cargo.

The declaration adds only `source_package` to the existing native fields:

```python
source_package = "dimos_lazy_native"
source_dir = "native"
executable = "target/release/package_probe"
build_command = "cargo build --release --locked --offline --bin package_probe --target-dir target"
```

Preparation snapshots this source directory into
`$XDG_CACHE_HOME/dimos/native-packages/<fingerprint>/source` (normally under
`~/.cache`). Both the build and native process use that snapshot as CWD. The
executable is relative to it; resources are declared with absolute paths to the
installed package. Builds never write into `site-packages`.

The source directory must contain its complete local build inputs, including any
workspace manifests and path dependencies. Local references outside this tree
are unsupported. Symlinks are rejected. `.git`, `.venv`, `__pycache__`, `target`,
`build` and `dist` directories are excluded from snapshots. Network dependencies
are controlled by the build command; this example uses `--locked --offline`.

Each source/recipe/environment fingerprint gets its own cache and process lock.
The fingerprint includes file bytes and modes, lockfiles, executable, command,
platform, explicit extra environment, common compiler settings and versions of
recognized toolchains. Editable source changes invalidate it without a version
bump. Arbitrary shell-command dependencies cannot all be inferred: after changing
an untracked external build input, stop dependent runs and use the existing
`--build-native` option to rebuild. Failed or interrupted builds have no completion
marker and retry from a fresh source snapshot. This is trusted build execution,
not a sandbox or signature check.

Stop dependent runs before uninstalling or removing a cache. Uninstall removes
the package's declarations and source files but leaves its regenerable cache and
user report. Existing checkout builds (`source_package=None`) and prebuilt
absolute executable paths keep their existing behavior, including checkout
fallback. No registry or new RPC is involved.
