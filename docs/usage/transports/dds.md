# Installing DDS Transport Libs on Ubuntu

The `dds` extra provides DDS (Data Distribution Service) transport support via [Eclipse Cyclone DDS](https://cyclonedds.io/docs/cyclonedds-python/latest/). The Python package builds C extensions against the CycloneDDS C library, so the C library must be installed before the Python package.

## Recommended: nix-provided cyclonedds

No `sudo`, no system pollution. Requires [Nix](/docs/installation/nix.md).

```bash
nix build nixpkgs#cyclonedds        # creates ./result symlink (GC root)
export CYCLONEDDS_HOME=$PWD/result
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH"
uv pip install -e '.[dds]'
```

`LD_LIBRARY_PATH` must stay set at runtime. Persist with one of:

```bash
# Per-venv (auto-set on `source .venv/bin/activate`)
cat >> .venv/bin/activate <<EOF
export CYCLONEDDS_HOME=$(readlink -f ./result)
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
EOF
```

```bash
# Global (every shell)
cat >> ~/.bashrc <<EOF
export CYCLONEDDS_HOME=$(readlink -f ./result)
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
EOF
```

## Alternative: Ubuntu apt + symlink shim

```bash
# Install the CycloneDDS development library
sudo apt install cyclonedds-dev

# Create a compatibility directory structure
# (required because Ubuntu's multiarch layout doesn't match the expected CMake layout)
sudo mkdir -p /opt/cyclonedds/{lib,bin,include}
sudo ln -sf /usr/lib/x86_64-linux-gnu/libddsc.so* /opt/cyclonedds/lib/
sudo ln -sf /usr/lib/x86_64-linux-gnu/libcycloneddsidl.so* /opt/cyclonedds/lib/
sudo ln -sf /usr/bin/idlc /opt/cyclonedds/bin/
sudo ln -sf /usr/bin/ddsperf /opt/cyclonedds/bin/
sudo ln -sf /usr/include/dds /opt/cyclonedds/include/

# Install with the dds extra
CYCLONEDDS_HOME=/opt/cyclonedds uv pip install -e '.[dds]'
```

To install all extras including DDS:

```bash
CYCLONEDDS_HOME=/opt/cyclonedds uv sync --extra dds
```
