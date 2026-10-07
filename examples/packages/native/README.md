# An installed native module

This independent project demonstrates the existing `NativeModule` and
`dimos.blueprints` interfaces. It builds a real C++ executable into a platform
wheel with scikit-build-core. No DimOS checkout is used by the installed module.
The small probe tests process/resource ownership; it does not implement pub/sub.

From this directory, with Python 3.10–3.12, CMake and a C++ compiler:

```bash
python -m pip install .
export DIMOS_PACKAGE_REPORT=/tmp/dimos-package-report.txt
dimos list
dimos --transport zenoh --viewer none run dimos-external-native.probe
```

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
a C++ compiler and uv in the test environment:

```bash
pytest dimos/core/test_external_native_package.py -m native_e2e --no-cov
```

It builds/installs the wheel into a temporary prefix, removes the package source,
checks metadata-only discovery and runs the real CLI from an unrelated directory.
