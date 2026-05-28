# DogOps SiteOps Agent

DogOps is an experimental Unitree Go2 SiteOps dashboard and MCP skill package.
It lives inside DimOS so the demo can run from one checkout without the
separate `dimos-unitree-go2` planning repo.

## Offline Smoke

```bash
uv run --no-sync python -m dimos.experimental.dogops.cli validate
uv run --no-sync pytest -q dimos/experimental/dogops
uv run --no-sync python -m dimos.experimental.dogops.cli simulate --out .dogops/runs/latest
uv run --no-sync python -m dimos.experimental.dogops.cli serve --run .dogops/runs/latest --host 127.0.0.1 --port 18768
```

Open `http://127.0.0.1:18768/`.

## Go2 Runtime

Use the real robot only after the base Go2 route and multicast route are known
good. Keep the stop command ready:

```bash
uv run --no-sync dimos stop --force
```

On macOS, DimOS LCM needs multicast routed to loopback:

```bash
sudo route delete -net 224.0.0.0/4 || true
sudo route add -net 224.0.0.0/4 -interface lo0
route -n get 224.0.0.1
```

`route -n get 224.0.0.1` must report `interface: lo0`. If it still reports a
VPN/tunnel interface such as `utun7`, disconnect that tunnel or remove its
multicast route before starting DimOS.

Start the DogOps Go2 stack:

```bash
GO2_IP=192.168.12.1 \
DOGOPS_ROBOT_IP=192.168.12.1 \
DOGOPS_SKIP_GO2_STARTUP_POSTURE=1 \
NO_PROXY=127.0.0.1,localhost \
no_proxy=127.0.0.1,localhost \
uv run --no-sync dimos --viewer none --rerun-open none --no-rerun-web run unitree-go2-dogops \
  -o go2connection.ip=192.168.12.1
```

In another shell:

```bash
NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost \
uv run --no-sync python -m dimos.experimental.dogops.cli serve --run .dogops/runs/latest --host 127.0.0.1 --port 18768
```

## Safe Route Workflow

Start with plain movement-only routes:

1. Confirm the dashboard shows live odom and costmap.
2. Create one waypoint within about `0.3m` to `0.5m` of the robot.
3. Run `Dry Run Route`.
4. Run `Run Live Route`.

Do not add `capture_image`, `gemini_inspect_image`, or `Gather Heatmap` to the
first hardware smoke. Camera/image actions are best-effort evidence steps and
should not be used as the first movement validation.

## Preflight Script

```bash
GO2_IP=192.168.12.1 scripts/dogops_go2_preflight.sh
```

By default this runs offline validation, registry checks, simulation, and an
optional robot ping. Hardware smoke is opt-in:

```bash
GO2_IP=192.168.12.1 RUN_GO2_SMOKE=1 scripts/dogops_go2_preflight.sh
GO2_IP=192.168.12.1 RUN_DOGOPS_SMOKE=1 scripts/dogops_go2_preflight.sh
```
