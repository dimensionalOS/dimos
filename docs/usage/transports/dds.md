# Installing DDS Transport Libs on Ubuntu

The `dds` extra provides DDS (Data Distribution Service) transport support via [Eclipse Cyclone DDS](https://cyclonedds.io/docs/cyclonedds-python/latest/). This requires installing system libraries before the Python package can be built.

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
