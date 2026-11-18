# Installing DDS Transport Libs on Ubuntu

The `dds` extra provides DDS (Data Distribution Service) transport support via [Eclipse Cyclone DDS](https://cyclonedds.io/docs/cyclonedds-python/latest/). On x86_64 Linux, macOS and Windows it installs from wheels that bundle the CycloneDDS C library, so nothing else is needed there:

```bash
uv sync --extra dds --inexact
```

## Linux arm64

There are no arm64 wheels, so the Python package builds C extensions against the CycloneDDS C library, and it needs the same release as the locked `cyclonedds` (11.1.0). Ubuntu's apt package (0.10) and nixpkgs (11.0.1) don't build with it, so build the C library from source:

```bash
sudo apt install -y build-essential cmake git
git clone --depth 1 -b 11.1.0 https://github.com/eclipse-cyclonedds/cyclonedds ~/cyclonedds-src
cmake -S ~/cyclonedds-src -B ~/cyclonedds-src/build -DCMAKE_INSTALL_PREFIX="$HOME/cyclonedds-11" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SSL=OFF -DENABLE_SECURITY=OFF
cmake --build ~/cyclonedds-src/build --target install -j"$(nproc)"
export CYCLONEDDS_HOME="$HOME/cyclonedds-11"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
uv sync --extra dds --inexact
```

`CYCLONEDDS_HOME` and `LD_LIBRARY_PATH` must stay set at runtime. Persist with one of:

```bash
# Per-venv (auto-set on `source .venv/bin/activate`)
cat >> .venv/bin/activate <<'EOF'
export CYCLONEDDS_HOME="$HOME/cyclonedds-11"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
EOF
```

```bash
# Global (every shell)
cat >> ~/.bashrc <<'EOF'
export CYCLONEDDS_HOME="$HOME/cyclonedds-11"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
EOF
```
