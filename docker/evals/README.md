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

## Using it on your branch

Nothing here depends on a particular suite or agent. On a branch based on a
main that has this, it is already there. Before that, take its commits:

```bash
git fetch origin ruthwik/feat/docker-evals && git cherry-pick origin/main..origin/ruthwik/feat/docker-evals
```

The one file an evals branch is likely to touch as well is
`dimos/evals/cli.py`, where this adds the `--docker` option to `run`; keep
both sets of options if it conflicts.

On the instance, the image is a snapshot of the checkout, so each branch
needs its own build. Tag it and point `EVALS_IMAGE` at it, so a rebuild for
one branch never replaces the image another person's eval is running on:

```bash
docker build -f docker/evals/Dockerfile -t dimensional/evals:my-branch .
export EVALS_IMAGE=dimensional/evals:my-branch
```

Rebuild after code changes; the dependency layer is cached unless the lock
file moved, so that is minutes, not the first build's half hour.

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
  needs the GPU overlay from the EC2 runbook below. It cannot run on a
  CPU-only host.
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

## EC2 runbook

Verified on a `g6.8xlarge` (32 vCPU, 128 GB, one L4) with plain Ubuntu 26.04.
A Deep Learning AMI already has the driver, Docker and the container toolkit;
skip what you have.

1. Driver, Docker with its compose and buildx plugins (Ubuntu's `docker.io`
   ships neither), git-lfs; then reboot for the driver:

   ```bash
   sudo apt-get update && sudo apt-get install -y nvidia-driver-580-server docker.io docker-compose-v2 docker-buildx git git-lfs && sudo usermod -aG docker "$USER" && sudo reboot
   ```

2. NVIDIA container toolkit, and BuildKit as Docker's default builder (the
   image uses cache mounts):

   ```bash
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   echo '{"features":{"buildkit":true},"runtimes":{"nvidia":{"path":"nvidia-container-runtime","runtimeArgs":[]}}}' | sudo tee /etc/docker/daemon.json && sudo systemctl restart docker
   docker run --rm --gpus all ubuntu:24.04 nvidia-smi
   ```

3. A headless Xorg on the GPU. DimSim's Chromium gets hardware WebGL on
   Linux only through an X server on the card (the self-hosted CI runner
   does the same; a virtual framebuffer is not enough). The datacenter GPU
   presents a virtual display of its own, so do not pass
   `--use-display-device=None`:

   ```bash
   sudo apt-get install -y xserver-xorg xinit x11-xserver-utils
   sudo nvidia-xconfig --allow-empty-initial-configuration --virtual=1280x720 --busid="PCI:$(nvidia-smi --query-gpu=pci.bus_id --format=csv,noheader | awk -F'[:.]' '{printf "%d:%d:%d", strtonum("0x"$2), strtonum("0x"$3), strtonum("0x"$4)}')"
   sudo tee /etc/systemd/system/xorg.service >/dev/null <<'UNIT'
   [Unit]
   Description=Headless Xorg on the GPU
   After=multi-user.target
   [Service]
   ExecStart=/usr/bin/Xorg :0 -noreset
   Restart=always
   [Install]
   WantedBy=multi-user.target
   UNIT
   sudo systemctl enable --now xorg && sleep 3 && sudo DISPLAY=:0 xhost +local:root
   ```

   `xhost` printing "non-network local connections being added" means Xorg
   is up; `sudo journalctl -u xorg -n 30` otherwise.

4. The socket buffers dimos's LCM setup asks for; host-wide, so they cannot
   be set per container:

   ```bash
   echo -e "net.core.rmem_max=67108864\nnet.core.rmem_default=67108864" | sudo tee /etc/sysctl.d/90-dimos.conf && sudo sysctl --system
   ```

5. The repo, its DimSim assets, and the host venv the `--docker` flag runs
   from (a few packages build from source, hence the compilers):

   ```bash
   sudo apt-get install -y build-essential pkg-config portaudio19-dev libturbojpeg0-dev libgl1
   curl -LsSf https://astral.sh/uv/install.sh | sh && source ~/.bashrc
   git clone -b <branch> https://github.com/dimensionalOS/dimos.git ~/dimos && cd ~/dimos && git lfs pull --include="misc/DimSim/**" && uv sync
   ```

6. The image, 30 to 40 minutes the first time:

   ```bash
   cd ~/dimos && docker build -f docker/evals/Dockerfile -t dimensional/evals .
   ```

7. Environment, in the shell profile so it survives reconnects. Habitat runs
   need `DIMOS_TRANSPORT=zenoh`; DimSim runs need it unset:

   ```bash
   export OPENAI_API_KEY=... TYPESAFE_API_KEY=... DISPLAY=:0 HABITAT_DATA_DIR=/data/habitat
   export COMPOSE_FILE=docker/evals/compose.yaml:docker/evals/compose.gpu.yaml:docker/evals/compose.habitat-data.yaml
   ```

8. Datasets into `HABITAT_DATA_DIR` (the overlay replaces the container's
   whole data folder, so the HM3D example goes there too):

   ```bash
   sudo mkdir -p /data/habitat && sudo chown "$USER" /data/habitat
   cd ~/dimos && docker compose run --rm worker /app/target/habitat/env/bin/python -m habitat_sim.utils.datasets_download --uids hm3d_example --data-path /app/target/habitat/data --no-replace
   uv tool install huggingface_hub && huggingface-cli login   # after accepting the terms on the hssd/hssd-hab dataset page
   huggingface-cli download hssd/hssd-hab --repo-type dataset --local-dir /data/habitat/hssd-hab
   ```

9. Check a container sees everything, then run:

   ```bash
   cd ~/dimos && docker compose run --rm worker bash -c 'nvidia-smi -L; echo DISPLAY=$DISPLAY; ls /tmp/.X11-unix'
   uv run dimos evals run --docker <suite> --agent <agent> ...
   ```

Sizing: with hardware rendering plan on 3 to 4 vCPU, 4 to 5 GB of RAM and a
slice of the one GPU per concurrent eval; start a `g6.8xlarge` at six and
watch per-case duration, score and `nvidia-smi` as you add more. The sim runs
on wall clock, so a starved GPU or CPU shows up as lower scores, not just
slower runs. Multi-GPU instances need one X screen per GPU and a
per-container `DISPLAY`, which the compose files do not do yet.

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
