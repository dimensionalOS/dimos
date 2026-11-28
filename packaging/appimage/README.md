# DimOS AppImage

One file that installs DimOS on Ubuntu 26.04, x86_64, with an NVIDIA GPU. Installing
means extracting it. After that DimOS runs offline, with nothing on the machine but
the NVIDIA driver: no apt packages, no git, no Nix, no compiler.

The image is about 6 GB. The extracted tree is about 15 GB.

## Install

```sh
sudo sh install.sh dimos-0.0.16-x86_64.AppImage
dimos --simulation run unitree-go2
```

[`install.sh`](install.sh) extracts the image to `/opt/dimos/<version>`, points
`/opt/dimos/current` at it and links `/usr/local/bin/dimos` to its `AppRun`. This
needs no FUSE. During the install the disk holds the image and the tree.

Nothing is written into `/opt/dimos` afterwards. Logs and other output go to
`~/.local/state/dimos`. API keys are read from the environment or from `.env` in the
working directory.

`AppRun` takes the arguments of `dimos`. Two more ways to call it:

- `AppRun bundle-exec <command> ...` runs any command with the image's environment,
  for example `bundle-exec python3 script.py`.
- A link to `AppRun` that is named after a command of the image runs that command:
  `ln -s /opt/dimos/current/AppRun /usr/local/bin/dtop`.

### Update and going back

Run `install.sh` with the newer image. It extracts next to the old version and
switches the link. Then restart DimOS. To go back, switch the link again:

```sh
sudo ln -sfn 0.0.16 /opt/dimos/current
```

### As a service

```ini
[Service]
User=dimos
ExecStart=/opt/dimos/current/AppRun --viewer none run unitree-go2
WorkingDirectory=/etc/dimos
KillSignal=SIGINT
LimitMEMLOCK=infinity
```

`.env` in `WorkingDirectory` holds the keys. Do not use `--daemon` in a unit.

### Running the file without extracting it

`./dimos-0.0.16-x86_64.AppImage --help` mounts the image and runs from it. This needs
`fuse3`. It is fine for a quick look and it is not the supported way: every start is
about 1.3 s slower, and systemd kills the mount under DimOS when it stops a unit.

## What the target needs

| Target | Needs |
|---|---|
| Ubuntu 26.04 desktop | The NVIDIA driver. |
| Ubuntu 26.04 server or container | The driver, and `libglib2.0-0t64 libgl1 libegl1 libxcb1 libx11-6 libxext6 libsm6 libice6 libgomp1 libasound2t64 libusb-1.0-0`. |

Ubuntu 24.04 and older are not supported. The RealSense, Go2 DDS and dim_slam native
modules need glibc 2.42.

## What is inside

| Path | Content |
|---|---|
| `AppRun` | Sets the environment and runs `dimos`. |
| `opt/python` | CPython from uv, the locked packages of the `all` and `learning` extras, the dimos wheel, the PyO3 extensions. |
| `opt/native/<name>/` | Native modules with their libraries. `DIMOS_NATIVE_DIR` |
| `opt/data` | The archives of [`data.tsv`](data.tsv), unpacked. `DIMOS_DATA_DIR` |
| `opt/models` | A Hugging Face cache with the models of [`models.tsv`](models.tsv). `HF_HOME` |
| `opt/robot_assets` | The robot descriptions of [`robot_descriptions.tsv`](robot_descriptions.tsv). `DIMOS_ROBOT_ASSETS_DIR` |
| `opt/deno-dir`, `usr/bin/deno` | Deno and its module cache, for the relay and the cockpit. |
| `usr/lib` | PortAudio, TurboJPEG, quadmath and a link to Open3D's TBB for Numba. |
| `licences` | What the image redistributes, and under which terms. |

`AppRun` sets the variables in the table, and `HF_HUB_OFFLINE=1`. A value that is
already set wins. Examples:

- Recordings are not in the image. Copy `opt/data` and the recordings into one folder
  and set `DIMOS_DATA_DIR` to it.
- A model that is not in the image: set `HF_HUB_OFFLINE=0` and `HF_HOME` to a folder
  that can be written. The models of the image are then downloaded again.

A robot description that is not in the image is cloned into `~/.cache/dimos` on first
use. That needs git and network.

Not in the image: DimSim, Ultralytics (YOLO, YOLOE and FastSAM, which are AGPL-3.0),
recordings, the `dds` and `unitree-dds` extras, the `dimos agent` Node harness and the
isolated Python modules under `native/python`.

Native modules that pick a build by machine have one build here: dim_slam for
CUDA 12, depth2depth_cloud for the CPU, the V4L2 camera without the Jetson encoder.
dim_slam uses the CUDA libraries of the NVIDIA wheels. The image has one copy of CUDA.

## Build

```sh
DIMOS_APPIMAGE_OUT=/mnt/appimage packaging/appimage/build.sh
```

The result is `<out>/dimos-<version>-<arch>.AppImage` and its `.sha256` file. CI
builds it with the [`appimage` workflow](/.github/workflows/appimage.yml), started by
hand. The workflow's artifact holds the image, the `.sha256` file and `install.sh`.
Check a copy with `sha256sum -c dimos-<version>-<arch>.AppImage.sha256`.

- **Run it from a clean checkout.** The wheel is built from the files that are there.
- **Use a neutral output folder.** The path ends up in package metadata.
- The build machine needs `uv`, `git` with LFS, `nix`, `cargo`, a C++ compiler,
  `mksquashfs`, `curl`, `unzip` and `docker`. It needs PortAudio, TurboJPEG and
  quadmath installed (`portaudio19-dev libturbojpeg libquadmath0` on Ubuntu 24.04).
  Its glibc must not be newer than the target's.
- It needs network: Python packages, the Nix cache, LFS, Hugging Face, GitHub.
- It needs about 50 GB of disk.
- With warm caches it takes about 11 minutes on a 32-thread machine. Four of them
  are the compression.

`build.sh <stage>...` runs single stages. In order:

| Stage | What it does |
|---|---|
| `web` | Downloads the pinned Deno, builds the SDK and the cockpit. The wheel build copies them. |
| `python` | Copies the pinned uv-managed CPython. |
| `wheel` | Builds the dimos wheel and the PyO3 extension wheels. |
| `deps` | Installs the locked packages, then the wheels. Of `onnxruntime` and `onnxruntime-gpu` only the GPU one. |
| `relay` | Fills the Deno module cache of the relay. |
| `native` | Builds the native modules with Nix and copies them out of the Nix store. |
| `syslibs` | Copies PortAudio, TurboJPEG and quadmath from the build machine. Links Open3D's TBB, so Numba finds it. |
| `data` | Pulls the LFS archives of `data.tsv`, checks the list, unpacks them. Adds a licence file where an archive has none. |
| `models` | Downloads the models of `models.tsv` at their pinned commits. |
| `robots` | Clones the robot descriptions of `robot_descriptions.tsv`. |
| `licences` | Writes the `licences` folder and prints the packages to review. |
| `bytecode` | Compiles every Python file. An edited `.py` still wins over its `.pyc`. |
| `launcher` | Adds `AppRun`, the desktop entry and the icon. |
| `check` | The checks below. |
| `image` | Packs the tree with zstd 19 and enforces the size limit. |

### Checks

The `check` stage stops the build when:

- a staged data licence or the NVIDIA model attribution is missing,
- a console script or a `.pth` file holds a build path,
- two packages own `site-packages/onnxruntime`,
- Ultralytics or one of its weight files is in the tree,
- there is more than one `libcublasLt`,
- a file is an LFS pointer,
- the loader cannot resolve a native executable's libraries, a required wheel
  library is missing, or the tested Python packages or Numba's TBB backend fail to load,
- `dimos --help`, `dimos list` or the relay with the cockpit do not run there, without
  network.

The last two run in an `ubuntu:26.04` container ([`check_target.sh`](check_target.sh)).

## The lists

Each list has one row per thing the image redistributes, with its licence.

### Data

`data.tsv` lists the LFS archive stems proposed for the production image. Each
row identifies its consumer, the robot or subsystem owner who should review it,
the licence evidence, and any open question. An owner records their name or
review reference in `reviewed_by` after checking the need for the archive and
the terms for its contents. Ticket 04 records project approval for seven rows with
components marked `licence unspecified`. Their upstream terms are recorded
where known. Keep the applicable licence notices and attributions in the image.

Run the inventory check from the DimOS checkout:

```sh
python3 packaging/appimage/check_data.py
```

The total counts compressed payload bytes even in a checkout with Git LFS
pointers. The build runs the following command after fetching the listed
payloads. On success stdout contains one archive path per line, ready for
extraction into the image data directory. On failure the build must stop.

```sh
python3 packaging/appimage/check_data.py --build-inputs
```

**The build stops while a row is `pending`.**

MuJoCo provides the hardware-free installation check. Replay recordings,
including the default Go2 and G1 datasets, are supplied separately by the
operator. The image also omits the test assets and LCM fixture, office video,
optional scene packages, the non-commercial person mesh and Ultralytics weights.

### Models

`models.tsv` lists the Hugging Face models that blueprints load: repository, pinned
commit, licence, use. A blueprint that needs a model that is not listed fails at
start, because the image runs offline.

To add a model, add a row with the commit of its default branch. The build also
writes `refs/main` for each model. DimOS asks for the default branch, and offline
that lookup only works with the ref.

### Robot descriptions

`robot_descriptions.tsv` lists the Git repositories behind `RobotDescriptionSource`:
URL, commit, licence, robots. The build stops when no robot config names a listed
commit any more. So a config that moves to a new commit needs its row updated.

The M20 fetches its description in its own way and is not in the image.

## Licences

Everything in the image is redistributed. [`THIRD_PARTY.md`](THIRD_PARTY.md) and the
tables next to it are copied into the image's `licences` folder. The `licences` stage
prints the Python packages without a licence in their metadata, and those with
conditions on redistribution (GPL, LGPL, proprietary). **Nobody has reviewed that
list yet.** Review it before the image is published.
