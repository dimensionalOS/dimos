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

# Pinned to uv.lock: the native must speak the same zenoh wire version as the
# dimos peers, or SHM payloads arrive as unreadable handles. --no-deps skips
# lcm-dimos-fork, the LCM runtime; only dimos_lcm's pure-python encoders are needed.
./env/bin/pip install --no-input --no-deps "dimos-lcm==0.1.3"
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
