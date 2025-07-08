#!/usr/bin/env bash
# This script fixes the onnxruntime <--> onnxruntime-gpu package clash
# that occurs when chromadb and other dependencies require the CPU-only
# onnxruntime package. It removes onnxruntime and reinstalls the GPU version.
set -euo pipefail

: "${GPU_VER:=1.18.1}"

python - <<PY
import subprocess, sys, importlib.metadata as md

gpu_ver = "${GPU_VER}"

def has_dist(name):
    try:
        md.version(name)
        return True
    except md.PackageNotFoundError:
        return False

if has_dist("onnxruntime"):
    print("Removing CPU-only onnxruntime wheel …")
    subprocess.check_call([sys.executable, "-m", "pip", "uninstall", "-y", "onnxruntime"])

print(f"Reinstalling onnxruntime-gpu=={gpu_ver} …")
subprocess.check_call([
    sys.executable, "-m", "pip", "install",
    "--no-deps", "--force-reinstall", f"onnxruntime-gpu=={gpu_ver}"
])
PY
