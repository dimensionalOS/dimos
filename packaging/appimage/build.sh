#!/usr/bin/env bash
# Builds the DimOS AppImage: one file with the Python interpreter, the locked Python
# packages, the dimos wheel, the native modules, data, models and robot descriptions.
#
#   packaging/appimage/build.sh              run every stage
#   packaging/appimage/build.sh deps image   run only the named stages
#
# Run it from a clean checkout. DIMOS_APPIMAGE_OUT names the output folder (default:
# target/appimage). The result is <out>/dimos-<version>-<arch>.AppImage. See README.md.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
readonly REPO
readonly HERE="$REPO/packaging/appimage"
readonly OUT="${DIMOS_APPIMAGE_OUT:-$REPO/target/appimage}"
readonly APPDIR="$OUT/AppDir"
readonly CACHE="$OUT/cache"
readonly WHEELS="$OUT/wheels"
readonly PY="$APPDIR/opt/python/bin/python3"

# The patch version is pinned: uv would otherwise take the newest one it knows.
readonly PYTHON_VERSION="3.12.13"
# dds and unitree-dds are left out: cyclonedds has no wheel for this Python and its
# source build needs the CycloneDDS C library. yolo is left out: Ultralytics is AGPL-3.0.
readonly EXTRAS="all learning"
# Packages that Ubuntu 26.04 must have. The library check installs exactly these.
readonly TARGET_PACKAGES="libglib2.0-0t64 libgl1 libegl1 libxcb1 libx11-6 libxext6 libsm6 libice6 libgomp1 libasound2t64 libusb-1.0-0"
# The image must stay below this size.
readonly MAX_IMAGE_MB=6500

# The static runtime needs no libfuse on the target, only the fusermount3 binary.
readonly RUNTIME_URL="https://github.com/AppImage/type2-runtime/releases/download/20251108"

ARCH="$(uname -m)"
readonly ARCH
case "$ARCH" in
  x86_64)
    readonly LOADER="/lib64/ld-linux-x86-64.so.2"
    readonly LDCONFIG_TAG="x86-64"
    readonly RUNTIME_SHA256="2fca8b443c92510f1483a883f60061ad09b46b978b2631c807cd873a47ec260d"
    # Libraries the Python packages load that a stock Ubuntu desktop does not have.
    # libjack is only there because PortAudio links it.
    readonly SYSLIBS="libportaudio.so.2 libjack.so.0 libturbojpeg.so.0 libquadmath.so.0"
    # onnxruntime and onnxruntime-gpu are both locked and own the same folder.
    readonly SKIP_PACKAGES="onnxruntime"
    # The dim_slam build for the CUDA 12 of the NVIDIA wheels.
    readonly DIM_SLAM_PACKAGE="x86_64-cuda12"
    # dim_slam's CUDA libraries, as paths in the NVIDIA wheels (ticket 03 checked these six).
    readonly DIM_SLAM_CUDA="cublas/lib/libcublasLt.so.12 cublas/lib/libcublas.so.12 cusparse/lib/libcusparse.so.12 cusolver/lib/libcusolver.so.11 nvjitlink/lib/libnvJitLink.so.12 cuda_runtime/lib/libcudart.so.12"
    ;;
  *)
    echo "build.sh: no settings for $ARCH yet" >&2
    exit 1
    ;;
esac

log() { printf '\n==> %s\n' "$*"; }
die() { echo "build.sh: $*" >&2; exit 1; }
project_version() { sed -n 's/^version = "\(.*\)"/\1/p' "$REPO/pyproject.toml" | head -1; }
site_packages() { "$PY" -c 'import sysconfig; print(sysconfig.get_path("purelib"))'; }
patchelf() { uvx --quiet patchelf@0.19.3.0 "$@"; }
# download <url> <file>: keeps no partial file.
download() { [[ -f "$2" ]] || { curl -fL -o "$2.part" "$1" && mv "$2.part" "$2"; }; }
# rows <file.tsv>: the rows below the header.
rows() { tail -n +2 "$HERE/$1"; }

# The pinned Deno goes into the image. It also builds the SDK and the cockpit, which
# the wheel build copies. These are the commands of .github/actions/build-cockpit.
stage_web() {
  log "web: Deno, SDK and cockpit builds"
  local version zip
  version="$(sed -n 's/^DENO_VERSION = "\(.*\)"$/\1/p' "$REPO/dimos/utils/deno.py")"
  [[ -n "$version" ]] || die "no DENO_VERSION in dimos/utils/deno.py"
  zip="$CACHE/deno-$version-$ARCH.zip"
  download "https://github.com/denoland/deno/releases/download/$version/deno-$ARCH-unknown-linux-gnu.zip" "$zip"
  mkdir -p "$APPDIR/usr/bin"
  unzip -o -q "$zip" deno -d "$APPDIR/usr/bin"
  (
    # vite and friends stay in a cache outside the image.
    export PATH="$APPDIR/usr/bin:$PATH" DENO_DIR="$CACHE/deno-build"
    cd "$REPO/web" && deno install --frozen
    cd "$REPO/web/sdk" && deno task build
    cd "$REPO/web/cockpit" && deno task build
  )
}

stage_python() {
  log "python: uv-managed CPython $PYTHON_VERSION"
  local interpreter
  UV_PYTHON_INSTALL_DIR="$CACHE/python" uv python install --no-bin "$PYTHON_VERSION"
  interpreter="$(UV_PYTHON_INSTALL_DIR="$CACHE/python" uv python find --managed-python "$PYTHON_VERSION")"
  interpreter="$(readlink -f "$interpreter")"
  rm -rf "$APPDIR/opt/python"
  mkdir -p "$APPDIR/opt"
  cp -a "$(dirname "$(dirname "$interpreter")")" "$APPDIR/opt/python"
  # The marker would make uv refuse to install into this interpreter.
  rm -f "$APPDIR"/opt/python/lib/python*/EXTERNALLY-MANAGED
  "$PY" --version
}

stage_wheel() {
  log "wheel: dimos and the PyO3 extensions"
  local manifest module
  rm -rf "$WHEELS"
  # CIBUILDWHEEL: setup.py drops -march=native, so the A* extension runs on every CPU.
  CIBUILDWHEEL=1 uv build --wheel --python "$PY" --out-dir "$WHEELS" "$REPO"
  # Found the same way as bin/pyo3-extensions, with its own target folder per module.
  "$PY" "$REPO/bin/pyo3-extensions" --list | while read -r manifest module; do
    MATURIN_PEP517_ARGS="--target-dir $CACHE/pyo3/$module" \
      uv build --wheel --python "$PY" --out-dir "$WHEELS" "$REPO/$(dirname "$manifest")"
  done
  ls "$WHEELS"
}

stage_deps() {
  log "deps: locked Python packages, then the wheels"
  local name args=()
  for name in $EXTRAS; do args+=(--extra "$name"); done
  for name in $SKIP_PACKAGES; do args+=(--no-install-package "$name"); done
  # copy: files in the image must not be hard links into uv's cache.
  UV_PROJECT_ENVIRONMENT="$APPDIR/opt/python" UV_LINK_MODE=copy \
    uv sync --project "$REPO" --python "$PY" --frozen --no-default-groups --no-install-project "${args[@]}"
  UV_LINK_MODE=copy uv pip install --python "$PY" --no-deps "$WHEELS"/*.whl
  relocate_scripts
  "$PY" -P -c 'import dimos.navigation.go2.replanning_a_star.min_cost_astar_ext'
  "$PY" "$REPO/bin/pyo3-extensions" --list | while read -r _ name; do "$PY" -P -c "import $name"; done
}

# Console scripts start with the build folder's absolute path. Make it relative, so
# the tree runs from any folder.
relocate_scripts() {
  local script
  for script in "$APPDIR"/opt/python/bin/*; do
    [[ -f "$script" && ! -L "$script" ]] || continue
    head -c 2 "$script" | grep -q '^#!' || continue
    head -n 1 "$script" | grep -q "$APPDIR" || continue
    # The same launcher uv writes for `uv venv --relocatable`.
    sed -i "1c #!/bin/sh\n'''exec' \"\$(dirname -- \"\$(realpath -- \"\$0\")\")\"/'python3' \"\$0\" \"\$@\"\n' '''" "$script"
  done
}

# The relay runs offline from the module cache made here.
stage_relay() {
  log "relay: Deno module cache"
  local web
  web="$("$PY" -P -c 'from dimos.web.relay_bridge.locate import find_web_dir; print(find_web_dir())')"
  [[ "$web" == "$APPDIR"/* ]] || die "the relay was found outside the image: $web"
  rm -rf "$APPDIR/opt/deno-dir"
  DENO_DIR="$APPDIR/opt/deno-dir" "$APPDIR/usr/bin/deno" cache --frozen --node-modules-dir=none \
    --config "$web/deno.json" "$web/relay/main.ts"
}

# Native modules are Nix builds: result/bin/<name> names a loader and libraries in
# /nix/store. The image gets a copy with the system loader and, next to it, every
# library that is not part of glibc. The layout is the one DIMOS_NATIVE_DIR reads:
# <name>/bin/<name>.
stage_native() {
  log "native: Nix-built module binaries"
  "$REPO/bin/native-modules" --build
  (cd "$REPO/dimos/mapping/dim_slam/rust" && nix build -L "path:.#$DIM_SLAM_PACKAGE")
  local flake binary name dest lib
  rm -rf "$APPDIR/opt/native"
  : > "$OUT/native-libraries.tsv"
  git -C "$REPO" ls-files -z 'dimos/**/flake.nix' | while IFS= read -r -d '' flake; do
    for binary in "$REPO/$(dirname "$flake")"/result/bin/*; do
      [[ -f "$binary" ]] || continue
      name="$(basename "$binary")"
      dest="$APPDIR/opt/native/$name"
      mkdir -p "$dest/bin" "$dest/lib"
      # ldd resolves through the binary's own Nix loader, so the paths are the Nix ones.
      ldd "$binary" | awk '$3 ~ "^/nix/store/" && $3 !~ "-glibc-" {print $3}' |
        while read -r lib; do
          cp -L --no-preserve=mode "$lib" "$dest/lib/"
          printf '%s\t%s\t%s\n' "$name" "$(basename "$lib")" "$(cut -d/ -f4 <<<"$lib" | cut -d- -f2-)" >> "$OUT/native-libraries.tsv"
        done
      cp -L --no-preserve=mode "$binary" "$dest/bin/$name"
      chmod 755 "$dest/bin/$name"
      patchelf --set-interpreter "$LOADER" --set-rpath '$ORIGIN/../lib' "$dest/bin/$name"
      for lib in "$dest"/lib/*; do
        [[ ! -e "$lib" ]] || patchelf --set-rpath '$ORIGIN' "$lib"
      done
      echo "  $name: $(find "$dest/lib" -type f | wc -l) libraries"
    done
  done
  [[ -x "$APPDIR/opt/native/dim_slam/bin/dim_slam" ]] || die "no native modules were staged"
  link_cuda
}

# dim_slam's Nix build brings its own CUDA. The NVIDIA wheels hold the same libraries,
# so the image keeps the wheels' copy and links to it.
link_cuda() {
  local lib site dest="$APPDIR/opt/native/dim_slam/lib"
  site="$(site_packages)"
  for lib in $DIM_SLAM_CUDA; do
    [[ -f "$dest/$(basename "$lib")" ]] || die "dim_slam no longer loads $(basename "$lib")"
    [[ -f "$site/nvidia/$lib" ]] || die "no NVIDIA wheel has $lib"
    ln -sfr "$site/nvidia/$lib" "$dest/$(basename "$lib")"
    sed -i "/^dim_slam\t$(basename "$lib")\t/d" "$OUT/native-libraries.tsv"
  done
}

stage_syslibs() {
  log "syslibs: system libraries from the build machine"
  local soname path
  rm -rf "$APPDIR/usr/lib"
  mkdir -p "$APPDIR/usr/lib"
  : > "$OUT/system-libraries.tsv"
  for soname in $SYSLIBS; do
    # No early exit in awk: ldconfig would die of SIGPIPE and pipefail would stop the build.
    path="$(ldconfig -p | awk -v soname="$soname" -v tag="$LDCONFIG_TAG" '$1 == soname && index($0, tag) && !seen++ {print $NF}')"
    [[ -n "$path" ]] || die "$soname is not installed on this machine"
    cp -L "$path" "$APPDIR/usr/lib/$soname"
    printf '%s\t%s\n' "$soname" "$(dpkg -S "$(readlink -f "$path")" | cut -d: -f1)" >> "$OUT/system-libraries.tsv"
  done
  # sitecustomize.py makes ctypes.util.find_library see them.
  cp "$HERE/sitecustomize.py" "$(site_packages)/"
  ls "$APPDIR/usr/lib"
}

# check_data.py stops the build while a row of data.tsv is not reviewed.
stage_data() {
  log "data: the reviewed LFS archives"
  local archives archive
  git -C "$REPO" lfs pull --exclude="" \
    --include="$(rows data.tsv | cut -f1 | sed 's|.*|data/.lfs/&.tar.gz|' | paste -sd,)"
  archives="$("$PY" "$HERE/check_data.py" --build-inputs)"
  rm -rf "$APPDIR/opt/data"
  mkdir -p "$APPDIR/opt/data"
  while read -r archive; do
    tar -xzf "$archive" -C "$APPDIR/opt/data" --warning=no-unknown-keyword
  done <<<"$archives"
  /usr/bin/du -sh "$APPDIR/opt/data"
}

# A Hugging Face cache with the models of models.tsv, each at its pinned commit.
stage_models() {
  log "models: Hugging Face cache"
  local repo revision
  rm -rf "$APPDIR/opt/models"
  rows models.tsv | while IFS=$'\t' read -r repo revision _; do
    HF_HOME="$APPDIR/opt/models" "$PY" -P -c 'import sys
from huggingface_hub import snapshot_download
print(snapshot_download(sys.argv[1], revision=sys.argv[2]))' "$repo" "$revision"
    # A download by commit writes no ref. DimOS asks for the default branch, and an
    # offline lookup of that branch reads refs/main.
    mkdir -p "$APPDIR/opt/models/hub/models--${repo//\//--}/refs"
    printf '%s' "$revision" > "$APPDIR/opt/models/hub/models--${repo//\//--}/refs/main"
  done
  # Download bookkeeping, not part of the cache that is read.
  rm -rf "$APPDIR/opt/models/hub/.locks" "$APPDIR/opt/models/xet"
  /usr/bin/du -sh "$APPDIR/opt/models"
}

# The robot descriptions of robot_descriptions.tsv, in the layout that
# DIMOS_ROBOT_ASSETS_DIR reads. DimOS's own cache code makes the checkouts.
stage_robots() {
  log "robots: robot descriptions"
  local url ref site
  site="$(site_packages)"
  rm -rf "$APPDIR/opt/robot_assets"
  rows robot_descriptions.tsv | while IFS=$'\t' read -r url ref _; do
    grep -rqF "$ref" "$site/dimos/robot" || die "no robot config names $url at $ref any more"
    "$PY" -P -c 'import sys
from dimos.robot.assets.git_cache import GitAssetCache
print(GitAssetCache(sys.argv[1]).resolve(sys.argv[2], sys.argv[3]))' "$APPDIR/opt/robot_assets" "$url" "$ref"
  done
  # Only the files are read: no lock files, no Git history.
  rm -rf "$APPDIR/opt/robot_assets/locks"
  find "$APPDIR/opt/robot_assets/sources" -mindepth 3 -maxdepth 3 -name .git -exec rm -rf {} +
  /usr/bin/du -sh "$APPDIR/opt/robot_assets"
}

stage_licences() {
  log "licences: attribution and the tables of what the image redistributes"
  local package
  rm -rf "$APPDIR/licences"
  mkdir -p "$APPDIR/licences"
  cp "$REPO/LICENSE" "$HERE/THIRD_PARTY.md" "$HERE"/{data,models,robot_descriptions}.tsv \
    "$OUT"/{native,system}-libraries.tsv "$APPDIR/licences/"
  cut -f2 "$OUT/system-libraries.tsv" | sort -u | while read -r package; do
    cp "/usr/share/doc/$package/copyright" "$APPDIR/licences/$package.copyright"
  done
  "$PY" "$HERE/licences.py" "$(site_packages)" "$APPDIR/licences/python-packages.tsv"
}

# The installed tree is read-only, so Python cannot write .pyc files at run time.
# checked-hash: an edited .py is noticed and wins over its .pyc.
stage_bytecode() {
  log "bytecode: compile every Python file"
  "$PY" -m compileall -q -j 0 --invalidation-mode checked-hash -s "$APPDIR" \
    "$APPDIR/opt/python/lib"
}

stage_launcher() {
  log "launcher: AppRun, desktop entry, icon"
  install -m 755 "$HERE/AppRun" "$APPDIR/AppRun"
  cp "$REPO/assets/dimensional.svg" "$APPDIR/"
  # install.sh reads the version from here.
  sed "s/@VERSION@/$(project_version)/" "$HERE/dimos.desktop" > "$APPDIR/dimos.desktop"
}

stage_check() {
  log "check: the tree before it is packed"
  local site found
  site="$(site_packages)"
  if grep -rIl -e "$OUT" -e "$REPO" "$APPDIR/opt/python/bin" "$site"/*.pth; then
    die "the files above hold a build path"
  fi
  found="$(grep -l '^onnxruntime/__init__.py,' "$site"/*.dist-info/RECORD)"
  [[ "$(wc -l <<<"$found")" == 1 ]] || die "site-packages/onnxruntime needs one owner, got: $found"
  found="$(find "$APPDIR" \( -path "$site/ultralytics*" -o -iname 'yolo*.pt' -o -iname 'fastsam*.pt' -o -iname 'mobileclip*' \) -print -quit)"
  [[ -z "$found" ]] || die "Ultralytics is in the image: $found"
  found="$(find "$APPDIR" -type f -name 'libcublasLt.so*')"
  [[ "$(wc -l <<<"$found")" == 1 ]] || die "expected one libcublasLt, got: $found"
  found="$(find "$APPDIR" -type f -size -1024c -exec grep -lI -m1 '^version https://git-lfs.github.com/spec/' {} +)" || true
  [[ -z "$found" ]] || die "LFS pointers instead of files: $found"
  # Ubuntu 26.04 with the documented packages and no network after the install.
  docker build -q -t dimos-appimage-check - <<EOF
FROM ubuntu:26.04
RUN apt-get update && apt-get install -y --no-install-recommends $TARGET_PACKAGES && rm -rf /var/lib/apt/lists/*
EOF
  docker run --rm --network none -v "$APPDIR:/app:ro" -v "$HERE:/packaging:ro" \
    -v "$REPO/.github/scripts/wheel_smoke.py:/wheel_smoke.py:ro" dimos-appimage-check \
    bash /packaging/check_target.sh
}

stage_image() {
  log "image: squashfs + runtime"
  local runtime="$CACHE/runtime-$ARCH" image size
  image="$OUT/dimos-$(project_version)-$ARCH.AppImage"
  download "$RUNTIME_URL/runtime-$ARCH" "$runtime"
  echo "$RUNTIME_SHA256  $runtime" | sha256sum --check --quiet
  rm -f "$OUT/dimos.squashfs"
  # Keep these settings between releases: rsync onto an older image then only moves
  # what changed.
  mksquashfs "$APPDIR" "$OUT/dimos.squashfs" -comp zstd -Xcompression-level 19 -b 1M \
    -root-owned -noappend -no-progress
  cat "$runtime" "$OUT/dimos.squashfs" > "$image"
  rm "$OUT/dimos.squashfs"
  chmod 755 "$image"
  (cd "$OUT" && sha256sum "$(basename "$image")" > "$(basename "$image").sha256")
  size="$(( $(stat -c %s "$image") / 1024 / 1024 ))"
  echo "$image: $size MB"
  [[ "$size" -le "$MAX_IMAGE_MB" ]] || die "the image is $size MB, the limit is $MAX_IMAGE_MB MB"
}

readonly STAGES=(web python wheel deps relay native syslibs data models robots licences bytecode launcher check image)

main() {
  local stage
  mkdir -p "$CACHE"
  [[ $# -gt 0 ]] || set -- "${STAGES[@]}"
  for stage in "$@"; do
    [[ " ${STAGES[*]} " == *" $stage "* ]] || die "unknown stage '$stage' (stages: ${STAGES[*]})"
  done
  for stage in "$@"; do
    SECONDS=0
    "stage_$stage"
    echo "    ($stage: ${SECONDS}s)"
  done
}

main "$@"
