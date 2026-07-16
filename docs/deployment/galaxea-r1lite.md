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

### Public base, no private registry
```dockerfile
ARG ROS_BASE=ros:humble-ros-base-jammy
```
`ros-base`, not `desktop` — no GUI stack, no RViz, no Gazebo. A customer
robot can build or pull this with **zero credentials**. (Contrast: the dev
image is private ghcr, which is why installing it onboard needs a
`docker login` or a 15 GB `docker save | ssh` transfer.)

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

### Compose — two services
```yaml
network_mode: host   # DDS discovery/multicast with the vendor stack
ipc: host            # FastDDS same-host SHARED MEMORY
restart: unless-stopped
```
- **`network_mode: host`** — DDS multicast must reach the vendor stack.
- **`ipc: host`** — without it, FastDDS same-host shared memory fails and
  you get the signature symptom: **topics visible, zero messages**.
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
sudo vi /opt/dimos/.env                                  # DIMOS_IMAGE=<new tag>
docker compose -f /opt/dimos/compose.yaml up -d

# Rollback: put the old tag back, up -d again. Images are immutable
# and versioned, so rollback is always available.

docker compose -f /opt/dimos/compose.yaml logs -f dimos
docker compose -f /opt/dimos/compose.yaml ps
docker compose -f /opt/dimos/compose.yaml down          # remove
```
`.env` is written once and never overwritten — a re-run of `setup.sh` won't
clobber a robot's pinned version.

---

## 6. Gotchas — each one cost real time

| Symptom | Cause | Fix |
|---|---|---|
| Topics visible, **zero messages** | Container `/dev/shm` is private, or root container vs vendor's uid-1000 SHM segments | `ipc: host` (runtime) / UDP-only FastDDS profile (dev, `fastdds_udp_only.xml`) |
| `UNKNOWN-0.0.0` wheel, no packages | jammy setuptools 59 ignores `[project]` | pin `setuptools>=70` in builder |
| `TypeError` in `canonicalize_version` | setuptools 70 × jammy packaging 21.3 | pin `packaging>=24` |
| Illegal instruction on another robot | `-march=native` baked in | `CIBUILDWHEEL=1` |
| Build takes forever | `data/.lfs` in context | `build.sh` stages `git archive` + drops `data/` |
| `unrecognized image format` on load | password piped into `sudo` ate `docker load`'s stdin | don't pipe into sudo; `ssh r1lite "sudo -n docker load"` |
| `rclpy` not importable via `docker exec` | `bash -c` is non-interactive, skips `.bashrc` → ROS unsourced | source ROS explicitly (entrypoint does) |
| `RTNETLINK: Operation not permitted` | container has no `CAP_NET_ADMIN`; dimos' LCM configurator wants multicast on `lo` | apply on the **host**; does not persist across reboot |
| rerun web hangs | `rr.serve_grpc()` GIL-spin in forkserver workers | viewer as its own container/process |
| Arm commands silently overridden | factory GELLO teleop session holding the arms | `tmux kill-session -t r1lite_teleop` |
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

Hardware-validated on a real R1 Lite (2026-07): plain-ROS bring-up suite
passes onboard, and `r1lite-coordinator` runs against the live vendor stack.

Not yet validated: the runtime image rebuilt against current `main` — the
image predates a large rebase and the deps/viewer changes that came with it.
Rebuild and re-run `setup.sh`'s step-7 DDS check before trusting it on a
customer robot.

The full evidence trail — every hypothesis, test, and conclusion, including
the dead ends — is in
[`scripts/r1lite_test/BRINGUP_LOG.md`](../../scripts/r1lite_test/BRINGUP_LOG.md).
Live procedures are in
[`scripts/r1lite_test/RUNBOOK.md`](../../scripts/r1lite_test/RUNBOOK.md).
