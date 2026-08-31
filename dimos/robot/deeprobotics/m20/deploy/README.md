# M20 deployment

Run `deeprobotics-m20-kronknav-control` on the M20 development computer
(`10.21.31.104`).

## One-time robot setup

From the repository checkout on `10.21.31.104`, run:

```bash
./dimos/robot/deeprobotics/m20/deploy/setup.sh
```

The script configures the other onboard computer to forward lidar data and stop
its competing vendor planner, then configures this computer to expose the lidar
data to DimOS. It is safe to run again and persists across reboots.

## Run

```bash
cd /var/opt/robot/data/dimos-m20-kronknav
uv sync --extra all
source .venv/bin/activate
dimos --build-native --transport lcm --rerun-host 0.0.0.0 \
  run deeprobotics-m20-kronknav-control --daemon
```

Connect the viewer:

```bash
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

Attach the RPC shell:

```bash
dimos --transport lcm shell
```

Then stand the robot up or lie it down:

```python
app.M20Connection.standup()
app.M20Connection.liedown()
```
