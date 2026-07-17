# Deploying dimos on a Galaxea R1 Lite

How dimos runs on an R1 Lite onboard PC: the mental model, why the image is
built the way it is, and how to take a blank robot to driving.

Written to be portable — [Porting to another Galaxea robot](#porting-to-another-galaxea-robot-r1-pro)
at the end lists exactly what is R1 Lite-specific and what is not.

---

## 1. The mental model: three layers, one box

Everything runs on the robot's own PC. Nothing here needs a laptop once
installed.

```
┌──────────────────────────────────────────────────────────────────────┐
│  R1 Lite onboard PC  (Ubuntu 22.04, x86-64)                          │
│                                                                       │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │  LAYER 1 — Galaxea vendor stack  (host, NOT ours)              │  │
│  │  tmux sessions: hdas · mobiman · ros_discovery · system · tools │  │
│  │  Boots via ~/galaxea/.../robot_startup.sh (scripts/r1lite_test/ │  │
│  │  roslaunch.sh wraps it). Owns the motors. Talks ROS 2 Humble.   │  │
│  │                                                                 │  │
│  │      HDAS  ── arms · torso · grippers  (serial joints)          │  │
│  │      VCU   ── chassis  (holonomic swerve; RC-gated)             │  │
│  └───────────────┬────────────────────────────────────────────────┘  │
│                  │  ROS 2 Humble · DDS · ROS_DOMAIN_ID=2             │
│                  │                                                    │
│    feedback  ────┤  /hdas/feedback_arm_left|right                    │
│    (robot →)     │  /hdas/feedback_chassis · feedback_torso          │
│                  │  /hdas/feedback_gripper_left|right                │
│                  │  /hdas/imu_chassis · imu_torso                    │
│                  │                                                    │
│    commands  ────┤  /motion_target/target_speed_chassis              │
│    (→ robot)     │  /motion_target/target_joint_state_arm_left|right │
│                  │  /motion_target/target_position_gripper_*         │
│                  │                                                    │
│  ┌───────────────┴────────────────────────────────────────────────┐  │
│  │  LAYER 2 — dimos  (our container: dimos-r1lite)                │  │
│  │  network_mode: host · ipc: host                                 │  │
│  │                                                                 │  │
│  │   R1LiteConnection  ← THE ONLY MODULE THAT SPEAKS ROS           │  │
│  │        translates ROS  ⇄  LCM, owns the chassis dead-man        │  │
│  │              │                                                  │  │
│  │              │  LCM  (dimos' internal bus — never leaves ROS)   │  │
│  │              │  /r1lite/motor_states · /r1lite/motor_command    │  │
│  │              │  /chassis/cmd_vel · /cmd_vel  (public Twist bus) │  │
│  │              ▼                                                  │  │
│  │   ControlCoordinator  @100Hz                                    │  │
│  │        servo_r1lite  → 16-DOF upper body  (transport_lcm)       │  │
│  │        vel_chassis   → chassis/vx,vy,wz   (transport_lcm)       │  │
│  │              ▲                                                  │  │
│  │              │  twist_command (LCM /cmd_vel)                    │  │
│  │        any Twist publisher: KeyboardTeleop, nav, agents...      │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                                                                       │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │  LAYER 3 — viewer  (own container, same image)                 │  │
│  │  rerun --serve-web --port 9877   → browse from the laptop      │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

**The one idea worth internalising:** dimos does not speak ROS. Its internal
bus is LCM. `R1LiteConnection` is the single translation boundary — the only
place ROS appears. Everything above it (coordinator, tasks, teleop, nav)
is robot-agnostic and would work identically over any other transport.

That is why adding a robot to dimos is mostly *writing one connection
module*, not plumbing ROS through the whole stack.

### Where control actually flows

Driving the base with WASD, end to end:

```
KeyboardTeleop.cmd_vel
   → LCM /cmd_vel                       (public Twist bus)
   → ControlCoordinator.twist_command
   → vel_chassis task (JointVelocityTask: chassis/vx, vy, wz)
   → transport_lcm BASE adapter → LCM /chassis/cmd_vel
   → R1LiteConnection
   → ROS /motion_target/target_speed_chassis   (RELIABLE QoS, streamed)
   → VCU → wheels
```

`/cmd_vel` is deliberately pinned as a **public bus** in
`r1lite_coordinator.py` — both the coordinator's `twist_command` In and any
module's `cmd_vel` Out map to the same LCM topic. Any Twist publisher drives
the chassis with no extra wiring. That is the extension point for nav,
agents, or a new teleop source.

---

## 2. Two deployment paths — pick the right one

| | **Runtime** (`scripts/galaxea/`) | **Dev** (`scripts/r1lite_test/`) |
|---|---|---|
| Entry | `scripts/galaxea/r1lite/setup.sh` | `scripts/r1lite_test/r1lite_dimos_install.sh` |
| Image | `dimos-r1lite` — **5.9 GB**, public base | `ghcr.io/dimensionalos/ros-dev:dev` — **~15 GB**, private |
| dimos | baked in as a **wheel** | bind-mounted repo + `uv` venv |
| Python | system 3.10 (no venv) | `.venv` built to 3.10 |
| Edit code? | no — rebuild the image | yes — it's your live checkout |
| Lifecycle | `docker compose`, `restart: unless-stopped` | manual `docker exec` |
| For | **fleet / customer robots** | bring-up and debugging |

**Use the runtime path for anything that isn't active development.** It is
versioned, rollback-able, boot-survivable, and needs no credentials.

The rest of this document is the runtime path.

---

## 3. The image: every design decision and why

`scripts/galaxea/docker/Dockerfile` — ~5.9 GB, two stages.

### Public base, pinned by digest
```dockerfile
ARG ROS_BASE=ros@sha256:afb40d6b…   # == ros:humble-ros-base-jammy, 2026-07-16
```
`ros-base`, not `desktop` — no GUI stack, no RViz, no Gazebo. A customer
robot can build or pull this with **zero credentials**. (Contrast: the dev
image is private ghcr, which is why installing it onboard needs a
`docker login` or a 15 GB `docker save | ssh` transfer.)

Pinned by **digest**, not tag. `ros:humble-ros-base-jammy` is mutable and is
rebuilt regularly, so a tag means "whatever upstream published that day" —
the same reproducibility hole as letting pip resolve dependencies, one layer
down. Reproducibility only counts if it holds all the way to the OS. To move
it forward deliberately: `docker pull` the tag, read
`docker image inspect --format '{{index .RepoDigests 0}}'`, paste, rebuild.

### Reproducibility: the whole chain must be pinned
Three references decide what a robot actually runs, and **all three must be
immutable** or the guarantee is theatre:

| Layer | Pinned by | If left loose |
|---|---|---|
| OS + ROS | `ARG ROS_BASE=ros@sha256:…` | base drifts under you |
| ~150 Python deps | `uv.lock` → exported → `--no-deps` | pip takes latest-wins |
| The image itself | `DIMOS_IMAGE=…@sha256:…` in `.env` | a tag can be pushed over |

This was learned the hard way: the image once shipped `typer 0.27.0` against a
lock pinning `0.23.1`, because `pip` neither reads `uv.lock` nor honours
pyproject's `exclude-newer = "7 days"`. Every dimos command in that image died
with `TypeError: issubclass() arg 1 must be a class`.

### Builder stage — compile the wheel, throw the stage away
```dockerfile
ENV CIBUILDWHEEL=1
RUN pip install "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src
```
Four things here are load-bearing, each bought with a broken build:

- **`CIBUILDWHEEL=1`** drops `-march=native` from the C++ extension. Without
  it the wheel is compiled for *this* CPU and may crash (illegal
  instruction) on a different customer machine. Same switch the PyPI
  release uses.
- **`setuptools>=70`** — jammy ships setuptools 59, which silently ignores
  pyproject's `[project]` table and emits an `UNKNOWN-0.0.0` wheel
  containing nothing. dimos has no `__init__.py` files, so the package name
  and contents *must* come from pyproject.
- **`packaging>=24`** — setuptools 70 calls `canonicalize_version()` with a
  kwarg jammy's packaging 21.3 lacks → `TypeError` at metadata generation.
- **`--no-build-isolation`** — we just pinned the build deps deliberately;
  isolation would discard them.

The compiler toolchain never reaches the runtime image.

### Runtime stage — wheel + the smallest possible apt set
```dockerfile
RUN apt-get install -y --no-install-recommends \
        python3-pip python-is-python3 \
        libturbojpeg0-dev liblcm-dev libgl1 libglib2.0-0 iproute2
COPY --from=builder /wheels /tmp/wheels
RUN pip install /tmp/wheels/dimos-*.whl pygame && rm -rf /tmp/wheels
```
| apt package | why |
|---|---|
| `libturbojpeg0-dev` | PyTurboJPEG — camera stream encode |
| `liblcm-dev` | dimos' internal bus |
| `libgl1`, `libglib2.0-0` | opencv/open3d import-time shared libs |
| `iproute2` | `ip` — DDS/LCM interface checks |
| `python-is-python3` | `python` → `python3` for scripts |

**`pip install dimos-*.whl` installs core dependencies only — no extras.**
No `unitree`, no `manipulation`, no `cpu`/`cuda`, no `sim`, no torch, no
onnxruntime, no gtsam. `pygame` is added explicitly because
`r1lite-keyboard-teleop` needs it and it is otherwise only packaged in the
heavy `sim` extra.

This is what keeps it at 5.9 GB instead of ~15 GB. The remaining bulk is
dimos' core deps (open3d, opencv-contrib, numba/llvmlite, scipy,
pinocchio, rerun-sdk) — trimming further means changing dimos core, not
this Dockerfile.

> Note: `rerun-sdk` and `dimos-viewer` are currently **core** deps (there is
> a `TODO` in pyproject saying they shouldn't be). That's what lets the
> viewer service run from this same image. If rerun ever becomes optional,
> the viewer service needs an extra.

### No venv — on purpose
dimos is installed into the image's **system Python 3.10**, which *is*
Humble's rclpy Python (cp310 ABI). One interpreter, so the
`ImportError: rclpy is not installed` class of bug is structurally
impossible. The dev path needs `.envrc.humble` + `UV_PYTHON=3.10` precisely
because it has a venv that can drift; the runtime image doesn't.

### Entrypoint — boot-order-proof
`entrypoint.sh` sources ROS, then (for `run ...`) polls up to 120 s for
`/hdas/*` before launching. Combined with `restart: unless-stopped`, the
container may boot before the vendor stack and still come up correctly.
Skip with `DIMOS_NO_WAIT=1`. It `exec dimos "$@"`, so
`docker run <image> list` == `dimos list`.

### Build context — the 33 GB trap
```bash
./scripts/galaxea/docker/build.sh [revision]
```
It stages `git archive HEAD` into a temp dir and **deletes `data/`** before
building. Building from the repo root instead ships ~25–33 GB of LFS assets
to the daemon (`.dockerignore` re-includes `data/.lfs`), costing ~100 s per
build for files no R1 Lite blueprint loads. `--network=host` is used because
guest/corp networks often block docker's default DNS.

Tag: `dimos-r1lite:<pyproject-version>-r1lite.<rev>`, e.g.
`dimos-r1lite:0.0.11-r1lite.1`. **Builds the last commit — uncommitted
changes are not included.**

### Runs as uid 1000, not root
```dockerfile
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
```
Not only hygiene — root is a **data-loss bug** here. FastDDS delivers same-host
data by writing into the **reader's** `/dev/shm` segment. The vendor stack runs
as `r1lite` (uid 1000); a root container creates root-owned reader segments its
uid-1000 publishers cannot write into. Discovery still works over UDP, so
topics are visible and **not one message arrives** — silently, no error.

Matching the uid keeps zero-copy shared memory, which matters for the camera
streams. (The dev path instead uses a UDP-only FastDDS profile — uid-agnostic,
but it gives up zero-copy.) `setup.sh` writes `DIMOS_UID`/`DIMOS_GID` from
`id -u`/`id -g`, so it stays correct on a robot that ships a different uid.

A non-1000 uid must also be able to write `$HOME` — dimos keeps logs and its
run registry under `$HOME/.local/state/dimos` (`dimos/constants.py: STATE_DIR`).

### Build-time smoke test
```dockerfile
RUN dimos list | grep -q r1lite-coordinator
```
Placed **after `USER dimos`**, so it proves the image works as the runtime user
and can write its state dir. `dimos list` imports the whole blueprint registry —
it is exactly what caught the typer breakage. A broken image now **fails the
build** instead of reaching a robot.

### Compose — two services
```yaml
network_mode: host      # DDS discovery/multicast with the vendor stack
ipc: host               # FastDDS same-host SHARED MEMORY
user: "${DIMOS_UID:-1000}:${DIMOS_GID:-1000}"
restart: unless-stopped
stop_grace_period: 30s
logging: { driver: json-file, options: { max-size: 10m, max-file: "3" } }
```
- **`network_mode: host`** — DDS multicast must reach the vendor stack;
  docker's default bridge NAT drops it and no topics appear at all.
- **`ipc: host`** — without it, FastDDS same-host shared memory fails and you
  get the signature symptom: **topics visible, zero messages**. Necessary but
  **not sufficient** — see `user:` above.
- **`stop_grace_period: 30s`** — safety-critical. The chassis VCU **latches its
  last velocity target** and has no dead-man of its own; `R1LiteConnection`
  supplies one and sends a courtesy zero from `stop()`, which SIGTERM triggers.
  Docker's default grace is **10s, then SIGKILL** — and a killed process sends
  no zero, leaving a robot driving at its last commanded velocity. Teardown
  joins publisher threads and shuts the sensor executor down, so give it room.
- **`logging`** — docker's json-file driver is unbounded by default. An
  always-on 100Hz coordinator fills the disk and takes the robot down weeks
  later, at a customer site.
- **`healthcheck`** — a TCP connect to 7779, deliberately not an HTTP GET
  (`/` redirects to `/command-center`, which returns **503** unless the React
  app was built — it isn't in this image, so an HTTP probe would call a healthy
  robot unhealthy). Informational only: docker's restart policy does **not** act
  on health, so a false negative cannot restart-loop a working robot. It exists
  because `restart: unless-stopped` only catches a process that *exits*, never
  one that is alive but wedged.
- **viewer is a separate service** — dimos' in-process rerun web mode
  GIL-deadlocks inside forkserver workers (`rr.serve_grpc()` spins,
  starving worker 0; root-caused with py-spy, see BRINGUP_LOG). Running the
  rust server in its own process sidesteps it.

---

## 4. Blank R1 Lite → driving

**Prerequisites on the robot:** Ubuntu 22.04 x86-64, >20 GB free, the
Galaxea vendor stack installed and working (RC manual drives the base).

```bash
# 1. Get the repo (public — no credentials)
git clone https://github.com/dimensionalOS/dimos.git ~/dimos
cd ~/dimos

# 2. Bring the vendor stack up (idempotent)
./scripts/r1lite_test/roslaunch.sh

# 3. Install dimos
bash scripts/galaxea/r1lite/setup.sh
#    …or, if the robot has no registry access:
bash scripts/galaxea/r1lite/setup.sh --tar /path/to/dimos-r1lite.tar.gz
```

`setup.sh` is idempotent and prompts before every host change. It:

1. **Preflight** — arch + >20 GB free.
2. **Docker + compose** — installs `docker.io docker-compose-v2` if absent;
   falls back to `sudo docker` because the `docker` group isn't active until
   next login.
3. **Image ladder** — already present → registry pull → `--tar` load →
   build on the robot (~30–60 min).
4. **Sysctls** — `/etc/sysctl.d/60-dimos.conf`, 64 MB UDP read buffers for
   DDS/LCM.
5. **Deploy** — `/opt/dimos/{compose.yaml,.env}` + `/usr/local/bin/dimos`.
6. **Start** — `docker compose up -d`.
7. **Verify** — subscribes to `/hdas/feedback_arm_left` inside the container
   for 8 s. **>100 messages = the whole chain works.** This is the check that
   proves DDS actually crosses the container boundary, not just that things
   started.

To produce the tarball for an offline robot:
```bash
./scripts/galaxea/docker/build.sh
docker save dimos-r1lite:0.0.11-r1lite.1 | gzip > dimos-r1lite.tar.gz
```

### Then
```bash
dimos list                            # wrapper → runs in the container
dimos run r1lite-keyboard-teleop      # needs ssh -X (pygame window)
```
Browser viewer:
`http://<robot-ip>:9090?url=rerun%2Bhttp%3A%2F%2F<robot-ip>%3A9877%2Fproxy`

The coordinator runs as the always-on compose service. To run it in the
foreground instead:
`docker compose -f /opt/dimos/compose.yaml stop dimos`.

### Driving the chassis — the safety gate
The chassis will not move unless the **RC is ON with all 4 switches in
position 1 (= mode 5, "software may drive")**. RC OFF fails *safe* to mode 3,
which vetoes software and looks like a 0.3 mm/s creep.

⚠️ **The VCU latches the last target forever — there is no dead-man on the
robot side.** `R1LiteConnection` supplies one: it streams the chassis command
every tick and collapses to an explicit zero-velocity stream when `cmd_vel`
goes older than `cmd_vel_timeout_s` (0.3 s), and sends a courtesy zero on
shutdown. Never command the chassis from anything that doesn't stream.

⚠️ **Never power the robot on with an e-stop pressed.** It poisons the VCU
for the entire session — it ignores software, eventually kills RC manual too,
and survives stack restarts. Only a clean power cycle recovers it. If the
wheels refuse *both* software and RC manual: **power cycle, do not debug
software.**

---

## 5. Day-2 operations

```bash
# Update
sudo vi /opt/dimos/.env                          # DIMOS_IMAGE=<new digest or tag>
docker compose -f /opt/dimos/compose.yaml up -d

# Rollback: put the previous reference back, up -d again.

docker compose -f /opt/dimos/compose.yaml logs -f dimos
docker compose -f /opt/dimos/compose.yaml ps            # STATUS shows health
docker compose -f /opt/dimos/compose.yaml down          # remove
```
`.env` is written once and never overwritten — a re-run of `setup.sh` won't
clobber a robot's pinned version.

**Pin by digest, not tag.** `setup.sh` writes a digest automatically when the
image came from a registry:
```
DIMOS_IMAGE=ghcr.io/dimensionalos/dimos-r1lite@sha256:<digest>
```
Tags are **mutable** — anyone can push over `0.0.14b1-r1lite.1`, and then
"rollback to the known-good version" silently gets different bytes. A digest is
the only reference that means one exact image forever, which is what turns
rollback from a hope into a guarantee. An image loaded from a tarball or built
on the robot has no registry digest, so `setup.sh` falls back to the tag — which
is local and cannot be overwritten from outside anyway.

### Fleet upgrade

Because the image is immutable and the robot's version is one line of `.env`,
upgrading N robots is: publish a new tag → change one line per robot →
`up -d`. Robots can sit on different versions deliberately (canary one, hold
the rest). Nothing is built on a robot, and no robot's state depends on what
was on someone's laptop that day.

---

## 6. Gotchas — each one cost real time

| Symptom | Cause | Fix |
|---|---|---|
| Topics visible, **zero messages** | Container `/dev/shm` is private, **or** root container vs vendor's uid-1000 SHM segments (needs both fixes) | `ipc: host` **and** `user: 1000:1000` (runtime) / UDP-only FastDDS profile (dev, `fastdds_udp_only.xml`) |
| `TypeError: issubclass() arg 1 must be a class` on any `dimos` command | image built with pip resolving deps, ignoring `uv.lock` + `exclude-newer` → typer 0.27 vs the lock's 0.23.1 | install from the exported lock with `uv pip install --no-deps` |
| Robot fine for weeks, then disk full | docker's json-file logs are unbounded | `logging: max-size/max-file` in compose |
| Robot keeps driving after `compose down` | teardown exceeded docker's 10s grace → SIGKILL → no courtesy chassis zero | `stop_grace_period: 30s` |
| `ModuleNotFoundError: socketio` (or `starlette`) on any blueprint | core code imports them but pyproject declared them only in the `lint` group / `web` extra — invisible in a dev checkout, fatal in a core-only install | declared in core deps; the image's build-time smoke test now imports blueprints |
| Viewer shows old frames, `Latency: 45s`, GiBs of recording | six **uncompressed** streams logged unthrottled (dimos decodes the robot's JPEG to raw), and the proxy buffers 25% of RAM for late viewers | per-entity `max_hz` + `memory_limit: 1GB` in the blueprint's rerun config |
| Viewer connects but renders stale data; dimos `healthy` anyway | another process (e.g. a leftover dev container) already owns 9877, so the bridge never bound it — the healthcheck probes 7779, not rerun | one path per robot; check `sudo ss -tlnp \| grep 9877` is owned by the dimos container |
| `UNKNOWN-0.0.0` wheel, no packages | jammy setuptools 59 ignores `[project]` | pin `setuptools>=70` in builder |
| `TypeError` in `canonicalize_version` | setuptools 70 × jammy packaging 21.3 | pin `packaging>=24` |
| Illegal instruction on another robot | `-march=native` baked in | `CIBUILDWHEEL=1` |
| Build takes forever | `data/.lfs` in context | `build.sh` stages `git archive` + drops `data/` |
| `unrecognized image format` on load | password piped into `sudo` ate `docker load`'s stdin | don't pipe into sudo; `ssh r1lite "sudo -n docker load"` |
| `rclpy` not importable via `docker exec` | `bash -c` is non-interactive, skips `.bashrc` → ROS unsourced | source ROS explicitly (entrypoint does) |
| `RTNETLINK: Operation not permitted` | container has no `CAP_NET_ADMIN`; dimos' LCM configurator wants multicast on `lo` | apply on the **host**; does not persist across reboot |
| rerun web hangs | `rr.serve_grpc()` GIL-spin in forkserver workers | viewer as its own container/process |
| Arm commands silently overridden | factory GELLO teleop session holding the arms | `tmux kill-session -t r1lite_teleop` |
| `pygame.error: x11 not available` + `Authorization required` | X cookies are keyed by **(hostname, display)**; mounting `~/.Xauthority` isn't enough if the container's hostname differs — the cookie is addressed to someone else | create with `--hostname "$(hostname)"`; or `xhost +local:` on the host |
| `VIEWER=rerun-connect` fails at startup | viewer modes split; `GlobalConfig` is pydantic → validation error, not a warning | `VIEWER=rerun` + `RERUN_OPEN=none` |

---

## 7. Porting to another Galaxea robot (R1 Pro)

Most of this is already robot-agnostic. Concretely:

### Reusable as-is
- `scripts/galaxea/docker/Dockerfile` — only the final `CMD` is R1 Lite-specific
- `scripts/galaxea/docker/build.sh` — only the tag name
- `scripts/galaxea/docker/entrypoint.sh` — the `/hdas/*` wait is Galaxea-wide
- `scripts/galaxea/r1lite/compose.yaml` — only the `command:`
- `scripts/galaxea/r1lite/setup.sh` — only the tag + deploy dir
- `dimos-wrapper.sh` — verbatim

Suggested shape: `scripts/galaxea/r1pro/` alongside `r1lite/`, sharing
`scripts/galaxea/docker/`. Parameterise the blueprint via `ARG ROBOT` /
compose `command:` rather than forking the Dockerfile.

### What must be R1 Pro-specific
1. **The connection module** (`dimos/robot/galaxea/r1pro/connection.py`) —
   the real work. Topic names, DOF counts, units, QoS, and the dead-man.
2. **Joint list + hardware components** — R1 Lite is 16-DOF upper body
   (4 torso read-only + 2×6 arms) + 3-DOF holonomic chassis. R1 Pro differs.
3. **Units.** R1 Lite grippers are **0–100 native units, not radians/metres**
   — the R1 Pro catalog uses metres. Verify per robot; don't assume.
4. **Blueprint + registration** in `all_blueprints.py`.
5. **The bring-up ladder** — `scripts/r1pro_test/`, same shape: recon →
   topic discovery → read feedback → chassis → arm.

### Transferable lessons (do not re-learn these)
- **Prove the robot obeys plain ROS *before* introducing dimos.** The
  `test_0*.py` ladder exists for this. When something breaks later, you
  know which side it's on.
- **Command publishers need RELIABLE QoS.** A best-effort publisher cannot
  deliver to the RELIABLE subscriber the robot exposes on
  `target_speed_chassis`. A reliable publisher serves both kinds.
- **One-shot commands are ignored.** `ros2 topic pub --once` does nothing;
  the robot needs `-r 10`. Same reason `R1LiteConnection` streams.
- **Never joint-command a coupled linkage.** R1 Lite's torso is a
  parallelogram; single-joint deltas made it *shake*. The vendor default is
  `disable_torso=true` and joint+velocity torso signals conflict. Task-space
  (`target_speed_torso`) only, as a designed experiment. `test_06` is
  hard-guarded for this reason.
- **Read the vendor docs before commanding a new subsystem.** Every rule
  above was learned the expensive way.

### Python 3.10 is not negotiable
Humble's rclpy is cp310. dimos declares `requires-python = ">=3.10,<3.13"`,
but nothing in CI builds a 3.10 venv, so it rots. Three dependencies had to
be bounded to make it install at all (`onnxruntime`, `a750-control`,
`gtsam-extended` — all cp311/cp312-only wheels). The runtime image dodges
this by using system Python + core deps only; the **dev** path hits it head
on. Expect more of these over time, and check with a real 3.10 sync rather
than trusting a green CI.

---

## 8. Status

**Hardware-validated end-to-end on a real R1 Lite (2026-07-17).** The runtime
path in this document has been executed on the robot:

| | |
|---|---|
| Image built from current `main` + branch | ✅ `dimos-r1lite:0.0.14b1-r1lite.1` |
| `setup.sh --tar` on the robot | ✅ idempotent, prompts honoured |
| Step-7 DDS check | ✅ **1600 msgs/8s** (~185Hz, full rate) |
| Services | ✅ `dimos` **Up (healthy)**, `viewer` Up |
| Zero-copy shared memory as uid 1000 | ✅ no UDP-only profile needed |
| Live camera feed to a laptop viewer | ✅ no perceptible lag after throttling |
| Chassis driven under dimos (dev path) | ✅ WASD → `/cmd_vel` → VCU |
| **Teleop onboard from the runtime image** | ✅ `dimos run r1lite-keyboard-teleop` via the wrapper over `ssh -X` |

Known gaps:

- **Not published to a registry.** The image ships by `docker save | scp` +
  `setup.sh --tar`, so a laptop is still in the loop. Publishing to ghcr —
  ideally from CI on merge — is what makes a blank robot `git clone &&
  setup.sh`. See §4.
- **The WASD panel does not drive.** `RerunWebSocketServer` is composed and
  reachable, but `tele_cmd_vel` → `/cmd_vel` is unwired pending a measurement
  of the panel's baked-in speed (it is compiled into `dimos-viewer` and not
  tunable from Python).
- **Nothing in CI builds the 3.10 floor or a core-only install.** That is why
  this deployment kept finding packaging bugs — see §7.
- **Raw frames on the wire.** `R1LiteConnection` decodes the robot's JPEG to
  raw BGR, so rerun carries ~31 MB/s even throttled. Fine on the cable; the
  real fix (keep frames compressed for transport) matters for WiFi, more
  cameras, or higher rates.

> ⚠️ **Do not run the dev path and the runtime path on the same robot.** A
> leftover `dimos-dev-r1lite` container held ports 9877/9090 for 8 hours, so
> dimos never bound its gRPC port **while still reporting `healthy`** (the
> healthcheck probes 7779, not rerun) and a laptop viewer silently rendered
> stale code. `docker stop dimos-dev-r1lite` before deploying.

The full evidence trail — every hypothesis, test, and conclusion, including
the dead ends — is in
[`scripts/r1lite_test/BRINGUP_LOG.md`](../../scripts/r1lite_test/BRINGUP_LOG.md).
Live procedures are in
[`scripts/r1lite_test/RUNBOOK.md`](../../scripts/r1lite_test/RUNBOOK.md).
