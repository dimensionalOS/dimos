#!/usr/bin/env bash
# build_command for HabitatConnection: `nix develop path:. -c ./install.sh`.
# Builds into <repo>/target/habitat, outside the package tree. NativeModule
# treats the wrapper as the build sentinel, so it is written last.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../../.." && pwd)
OUT="$ROOT/target/habitat"
mkdir -p "$OUT"
cd "$OUT"

export MAMBA_ROOT_PREFIX="$OUT/mm"

if [ ! -x ./env/bin/python ]; then
    micromamba create -y -p ./env -c conda-forge -c aihabitat \
        python=3.9 habitat-sim headless withbullet
fi

# Build standalone generated CDR messages for Habitat's Python 3.9 interpreter.
# The DimOS checkout's environment generates the package; the native environment
# builds its own extension and never imports or installs DimOS at runtime.
if [ ! -x "$ROOT/.venv/bin/python" ]; then
    echo "Set up the DimOS project virtualenv before building Habitat messages." >&2
    exit 1
fi
if [ ! -f "$ROOT/build/message-codegen/install/lib/cmake/fastcdr/fastcdr-config.cmake" ]; then
    bash "$ROOT/scripts/setup_message_codegen.sh"
fi
(cd "$ROOT" && .venv/bin/python -m dimos.message_codegen.generate --package \
    --output "$OUT/messages" \
    --type geometry_msgs/msg/Twist --type nav_msgs/msg/Odometry \
    --type sensor_msgs/msg/CameraInfo --type sensor_msgs/msg/Image \
    --type sensor_msgs/msg/PointCloud2 --type tf2_msgs/msg/TFMessage)
CMAKE_PREFIX_PATH="$ROOT/build/message-codegen/install${CMAKE_PREFIX_PATH:+;$CMAKE_PREFIX_PATH}" \
    ./env/bin/pip install --no-input "$OUT/messages/python"
# Keep the native transport wire version pinned to uv.lock.
./env/bin/pip install --no-input "eclipse-zenoh==1.10.1" numpy

# Annotated HM3D house, no credentials. --no-replace resumes a partial download
# by the downloader's own per-package markers; a dir check would not.
./env/bin/python -m habitat_sim.utils.datasets_download \
    --uids hm3d_example --data-path ./data --no-replace

cat > habitat-native <<WRAP
#!/bin/sh
# NativeModule appends CLI args after the executable, so the script path cannot
# go in extra_args.
exec "$OUT/env/bin/python" "$HERE/../server.py" "\$@"
WRAP
chmod +x habitat-native
echo "built: $OUT/habitat-native"
