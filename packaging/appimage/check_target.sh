#!/usr/bin/env bash
# Checks the image tree on its target. build.sh runs this in an ubuntu:26.04
# container that has the documented packages and no network. The tree is at /app.

set -euo pipefail

# Libraries of optional features, which the target does not have: the NVIDIA driver,
# TensorRT and RDMA networking.
readonly OPTIONAL='^lib(cuda|nvinfer|nvonnxparser|mlx5|rdmacm|ibverbs)[.]so([.]|$)'

export LD_LIBRARY_PATH=/app/usr/lib
export LC_ALL=C
# Native executables must resolve their whole dependency tree from their installed path.
for binary in /app/opt/native/*/bin/* /app/usr/bin/deno /app/opt/python/bin/python3; do
  if ! deps="$(ldd "$binary" 2>&1)" || [[ "$deps" == *"not found"* ]]; then
    printf '%s\n%s\n' "$binary" "$deps" >&2
    exit 1
  fi
done

# Some wheels load their own libraries before importing extensions. Give ldd
# those package-specific paths, then exercise the real loaders in fresh processes.
site="$(/app/opt/python/bin/python3 -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
cuda_dirs=("$site"/nvidia/*/lib)
cuda_path="$(IFS=:; echo "${cuda_dirs[*]}")"
missing="$(find /app -type f \( -name '*.so' -o -name '*.so.*' -o -executable \) -print0 |
  while IFS= read -r -d '' file; do
    [[ "$(head -c 4 "$file" | tr -d '\0')" == $'\x7fELF' ]] || continue
    paths="$LD_LIBRARY_PATH"
    case "$file" in
      "$site"/*.libs/*) paths+=":${file%/*}" ;;
      "$site"/cmeel.prefix/*) paths+=":$site/cmeel.prefix/lib" ;;
      "$site"/mujoco/*) paths+=":$site/mujoco" ;;
      "$site"/onnxruntime/*) paths+=":$cuda_path" ;;
      "$site"/open3d/*) paths+=":$site/open3d:$site/torch/lib:$cuda_path" ;;
      "$site"/torchvision/*) paths+=":$site/torch/lib:$cuda_path" ;;
      */lib-dynload/_tkinter.*) paths+=":/app/opt/python/lib" ;;
    esac
    if ! deps="$(LD_LIBRARY_PATH="$paths" ldd "$file" 2>&1)"; then
      [[ "$deps" == *"not a dynamic executable"* ]] || printf '%s: %s\n' "$file" "$deps"
      continue
    fi
    [[ "$deps" == *"not found"* ]] || continue
    awk -v file="$file" -v optional="$OPTIONAL" '/not found/ && $1 !~ optional {print file ": " $0}' <<< "$deps"
  done)"
if [[ -n "$missing" ]]; then
  echo "These binaries need a library that the target does not have:"
  echo "$missing"
  exit 1
fi
for module in av cv2 ctranslate2 gtsam mujoco onnxruntime open3d PIL.Image pinocchio \
  pygame pydrake.all pxr.Usd scipy.linalg shapely tkinter torchvision; do
  /app/AppRun bundle-exec python3 -c "import $module"
done
NUMBA_THREADING_LAYER=tbb /app/AppRun bundle-exec python3 -c \
  'import numba; numba.get_num_threads(); assert numba.threading_layer() == "tbb"'
echo "native executables and Python package loaders passed"

/app/AppRun --help > /dev/null
/app/AppRun list > /dev/null
echo "dimos --help and dimos list run"
# Starts the relay from the installed wheel and fetches the cockpit from it.
/app/AppRun bundle-exec python3 /wheel_smoke.py

# Starts the simulation of the README without a window, then stops it. The start
# returns when every module runs. By then MuJoCo has loaded the packaged scene, robot
# and policy and has rendered a camera frame on the CPU. LCM gets an in-process URL:
# the container has no multicast route, and the blueprint's file makes an LCM
# transport when it is imported.
if ! { LCM_DEFAULT_URL=memq:// timeout 300 /app/AppRun --viewer none --simulation run unitree-go2 --daemon &&
  /app/AppRun stop; } > /tmp/simulation.log 2>&1; then
  cat /tmp/simulation.log
  exit 1
fi
echo "dimos --simulation run unitree-go2 starts and stops"
