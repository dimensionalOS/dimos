# An installed native module

This independent project demonstrates the existing `NativeModule` and
`dimos.blueprints` interfaces. It builds a standalone Rust executable into a platform
wheel with scikit-build-core. No DimOS checkout is used by the installed module.
The small probe tests process/resource ownership; it does not implement pub/sub,
use the DimOS native SDK, or connect to hardware. Cargo builds offline with a
committed lockfile and no third-party crates. CMake only invokes Cargo and installs
the resulting executable; no C/C++ compiler is needed.

Activate the coordinator's Python environment first. From this directory, with Python 3.10–3.12, uv, CMake and a Rust toolchain (`cargo`/`rustc`):

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" .
export DIMOS_PACKAGE_REPORT=/tmp/dimos-package-report.txt
dimos list
dimos --transport zenoh --viewer none --n-workers 1 --no-serve-coordinator-rpc run dimos-external-native.probe
```

In another terminal, `cat /tmp/dimos-package-report.txt` should show
`ready <PID> hello from an installed native package`. The same message appears in
the module log. Wait until the CLI reports that the blueprint is running before
pressing Ctrl-C. If prompted to apply system configuration changes, answer `n`;
this probe does not require those changes. Reading the report again should show a final `stopped` line.

The executable reads its packaged message resource and writes its PID/readiness
to the report. Ctrl-C stops it and appends `stopped`. Pick a report path owned by
your run. Install in the coordinator's Python environment, not a separate pipx
environment. The example currently targets POSIX systems.

The Python declaration passes absolute paths to the installed executable and
resource. `source_dir` and `build_command` remain unset: they describe builds in
the DimOS checkout, not arbitrary external projects. No public `cwd`, separate
registration command, or modification to `all_blueprints.py` is needed.

To build a distributable wheel/sdist use `python -m build`. A native executable
requires a platform wheel, even though it does not link to the CPython ABI.
`pip install -e .` uses the backend's editable behavior; rebuild native changes
and restart processes. Stop dependent runs before `pip uninstall
dimos-external-native`. The report is user data and is not removed by uninstall.

The explicit installation test requires `build`, `scikit-build-core`, CMake,
Cargo/Rust and uv in the test environment:

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" build scikit-build-core
# From the repository root, with the repository test dependencies installed:
python -m pytest dimos/core/test_external_native_package.py -m native_e2e --no-cov
```

It builds/installs the wheel into a temporary prefix, removes the package source,
checks metadata-only discovery and runs the real CLI from an unrelated directory.

Only the external example is installed as a platform wheel here. If the coordinator
uses an editable DimOS checkout, it still uses that checkout at runtime. This
example and its PR1 test do not claim to validate an installed DimOS host wheel.
Stop the foreground run before uninstalling with
`uv pip uninstall --python "$VIRTUAL_ENV/bin/python" dimos-external-native`.
This removes the example's executable, resource and entry point; it leaves DimOS,
the virtual environment, your report and normal build caches in place.
