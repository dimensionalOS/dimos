#!/usr/bin/env bash
# Build the habitat-sim environment and the wrapper NativeModule launches.
#
# Run through the flake: `nix develop path:. -c ./install.sh`, which is what
# HabitatNativeConfig.build_command does. NativeModule treats the wrapper's
# existence as the build sentinel, so it must be written last.
set -euo pipefail
cd "$(dirname "$0")"

export MAMBA_ROOT_PREFIX="$PWD/mm"

if [ ! -x ./env/bin/python ]; then
    micromamba create -y -p ./env -c conda-forge -c aihabitat \
        python=3.9 habitat-sim headless withbullet
fi

# dimos_lcm is the standalone message package (pure python, >=3.8) carrying the
# same lcm_encode payloads ZenohTransport puts on the wire, so the py3.9 side
# speaks dimos without importing dimos.
#
# --no-deps drops lcm-dimos-fork on purpose. That is the LCM *runtime* (a C/CMake
# build) and we publish over zenoh, so only the pure-python encoders are needed.
# Building it here also fails: nix develop is not a pure shell, CMake finds the
# host JDK, enables the lcm-java target and dies on a missing jchart2d jar.
./env/bin/pip install --no-input --no-deps dimos-lcm
./env/bin/pip install --no-input eclipse-zenoh numpy

# Scenes live beside the env so the module is self-contained: hm3d_example is
# a real annotated HM3D house (908 semantic instances) that needs no Matterport
# credentials. The licensed HM3D splits are a --uids change once a key exists.
if [ ! -d ./data/versioned_data/hm3d-0.2 ]; then
    ./env/bin/python -m habitat_sim.utils.datasets_download \
        --uids hm3d_example --data-path ./data
fi

cat > habitat-native <<'WRAP'
#!/bin/sh
# NativeModule always builds CLI args (stdin_config only adds the JSON line), so
# the script path cannot go in extra_args -- it would land after the flags.
here=$(dirname "$0")
exec "$here/env/bin/python" "$here/../server.py" "$@"
WRAP
chmod +x habitat-native
echo "built: $PWD/habitat-native"
