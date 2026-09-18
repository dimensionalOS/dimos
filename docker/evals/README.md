# Evals in Docker

One eval, one container. `dimos evals run --docker ...` starts a fresh worker
container from the eval image, runs the identical command inside it, and
returns at once. The container removes itself when the eval ends. Run the
command again for another suite and you have two evals running side by side;
each container has its own network namespace, so every dimos inside uses its
default ports and multicast groups without touching its neighbours.

```bash
dimos evals run --docker dimos.evals.suites.dimsim_apartment_qa \
    --agent dimos.evals.agents.pi --set max_steps=12 --limit 4
# evals-20260918-133000-dimsim_apartment_qa-3f1a: started (detached)
#   follow:   docker logs -f evals-20260918-133000-dimsim_apartment_qa-3f1a
#   results:  docker/evals/eval-runs/dimos/evals/   recordings + rerun.rrd: docker/evals/eval-runs/dimos/recordings/
```

`docker ps` lists the evals still running; `docker logs -f <name>` follows
one; `docker stop <name>` abandons one. Nothing in `dimos/evals` changes for
this: the container runs the stock `EvalRunner`, which boots a dimos plus
DimSim per case, runs the agent, records, and tears them down again.

## What a run leaves behind

Everything a container writes lands on the host under `EVAL_RUNS_DIR`
(default `docker/evals/eval-runs`), which the compose file mounts on `/state`:

```
eval-runs/dimos/evals/run-<stamp>-<id>/      the EvalRunner run dir: manifest, results.jsonl, summary.json, per-case trajectory
eval-runs/dimos/recordings/<run-id>/         one per dimos boot, i.e. one per case:
    memory.db                                the --record sensor recording the grades read
    rerun.rrd                                the whole Rerun stream of that case (see below)
```

Each case's result names its recording folder, so its `memory.db` and
`rerun.rrd` are one lookup away. Open a case's visualization with
`rerun eval-runs/dimos/recordings/<run-id>/rerun.rrd`.

## Rerun recording

Workers run with `VIEWER=rerun RERUN_OPEN=none RERUN_SAVE=1`: the Rerun
bridge logs as usual but never opens a viewer, and a headless `rerun --save`
client of the bridge's own gRPC server streams every event to `rerun.rrd`
next to that run's `memory.db`. The same `--rerun-save` flag works on a bare
`dimos run`, with a live viewer too if you want one. Files grow at roughly
20 MB per minute of a DimSim case with camera and lidar on, so budget disk
accordingly and sync the runs directory to S3 after a batch.

## Setup

Build the image once from the repo root, with the DimSim assets fetched (the
image bakes them, and the sim refuses to load LFS pointer stubs):

```bash
git lfs pull --include="misc/DimSim/**"
docker build -f docker/evals/Dockerfile -t dimensional/evals .
```

The image is a snapshot of the repo as it is on disk at build time, with the
venv, Deno, headless Chromium, the Pi agent CLI, and the DimSim frontend and
assets baked in. Rebuild after code changes; the dependency layer is cached
unless the lock file moved. API keys come from the host environment
(`OPENAI_API_KEY`, `ANTHROPIC_API_KEY`, `TYPESAFE_API_KEY`); export them
before running.

## Habitat

The image also carries Habitat: habitat-sim's own python 3.9 conda environment
under `target/habitat`, the annotated HM3D example house, and the
`habitat-native` wrapper, all built by the module's installer at image build
time so a run starts in seconds. Two things differ from DimSim runs:

- Habitat renders headless through EGL from the host's NVIDIA driver, so it
  needs the GPU overlay below. It cannot run on a CPU-only host.
- Its native process speaks zenoh, while DimSim needs LCM. Export
  `DIMOS_TRANSPORT=zenoh` for Habitat runs:

```bash
DIMOS_TRANSPORT=zenoh dimos evals run --docker dimos.evals.suites.habitat_smoke --agent dimos.evals.agents.pi
```

Only the HM3D example house is in the image. HSSD, ReplicaCAD and licensed
HM3D splits go on a host directory that `compose.habitat-data.yaml` mounts
over the container's `target/habitat/data`, so suites find them at their
default paths. Add it to `COMPOSE_FILE` next to the GPU overlay and set
`HABITAT_DATA_DIR`; the overlay's header shows how to fill the directory with
the image's own downloader.

## GPU rendering on EC2

Without the GPU overlay DimSim renders in software (`DIMSIM_RENDER=cpu`).
That works anywhere but is expensive: one software-rendered apartment scene
was measured at about 11 cores of Chromium on a 16-core laptop. For hardware
WebGL the containers render through an Xorg server on the host GPU, the same
way the self-hosted CI runner does; a virtual framebuffer is not enough.

Prepare the instance once (a `g6` class, Ubuntu 24.04):

1. Install the NVIDIA driver, Docker, and the NVIDIA container toolkit;
   run `nvidia-ctk runtime configure --runtime=docker` and restart Docker.
2. Configure a headless Xorg on the GPU with a virtual screen, for example
   `nvidia-xconfig --allow-empty-initial-configuration --use-display-device=None --virtual=1280x720`,
   and run `Xorg :0` under systemd so it survives logout.
3. Let containers use its socket: `xhost +local:root`.
4. Raise the socket buffers dimos asks for: `sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864`.

Then point every `--docker` run at both compose files:

```bash
export COMPOSE_FILE=docker/evals/compose.yaml:docker/evals/compose.gpu.yaml
dimos evals run --docker ...
```

Validate with one container and `--limit 1` first; the GPU path has only been
exercised on the CI runner so far. Then add containers while watching
per-case duration, score, and `nvidia-smi`. The sim runs on wall clock, so a
starved GPU or CPU shows up as lower scores, not just slower runs. Per
container with hardware rendering plan on 3 to 4 vCPU, 4 to 5 GB of RAM and a
slice of one GPU; a `g6.8xlarge` (32 vCPU, one L4) is a good first box at 6
concurrent evals.

## Pool mode (optional)

`dispatch.py` is the other way to use the same image: keep N idle containers
up and let one host command feed a suite's cases to them, one case per exec,
pulling the next case as each finishes.

```bash
docker compose -f docker/evals/compose.yaml up -d --scale worker=4
python docker/evals/dispatch.py --suite ... --agent ... [--set ...] [--tags ...] [--limit N]
```

It needs only Python and the docker CLI on the host, writes to
`eval-runs/pool/<run-id>/`, and merges every case into one summary. Use it
when one suite should finish as fast as possible; use `--docker` when you
want independent runs you start and forget.

## Notes

- Workers run dimos on LCM (`DIMOS_TRANSPORT=lcm`). DimSim's Deno bridge
  publishes odometry and takes velocity commands over LCM only; under the
  default zenoh transport the eval waits for odometry that never arrives.
- Workers get `NET_ADMIN` so dimos's own LCM setup can enable multicast on
  the loopback inside the container, and a raised `memlock` limit for zenoh's
  shared-memory pool.
- Everything in a container runs as root; the entrypoint opens permissions on
  what a run wrote so the host user can read and delete it.
- Run ids carry a random 4-hex suffix (`generate_run_id`). Two evals booting
  the same blueprint in the same second used to get the same run id and write
  one recording folder, one `memory.db` and one `rerun.rrd` between them.
- The dimos run registry (`/state/dimos/runs`) is an anonymous volume per
  container, not part of the shared mount. It is keyed by pid, and with a
  shared one a container's stale-entry sweep deletes its neighbours' live
  runs, whose evals then wait out their launch timeout looking for a
  recording.
- One Xorg display per host means one GPU per host. Multi-GPU instances need
  one X screen per GPU and a per-container `DISPLAY`, which the compose file
  does not do yet.
- Every concurrent eval calls the model API, so the account's rate limit caps
  useful parallelism as much as hardware does.
