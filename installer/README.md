# dimup

Prepare a machine for DimOS SDK development:

```bash
uv tool install --python 3.12 ./installer
dimup setup
```

This package is independent of the DimOS runtime. It supports Ubuntu 22.04/24.04
x86_64 and Apple Silicon macOS 14+. Setup installs host packages with apt/Homebrew,
prepares Cargo and Nix, and writes a detailed log. It is safe to rerun.

The release bootstrap installs uv and this tool, then runs setup. It does not
install a global `dimos` runtime. Release publishing is handled by DimOS CI.
