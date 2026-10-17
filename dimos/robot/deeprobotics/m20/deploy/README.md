# M20 deployment

Run setup and the complete DimOS stack, including the Rerun bridge, on GOS
(`10.21.31.104`). Setup builds and installs the NOS bridge, runs the RoboSense
driver beside vendor LIO on NOS, and disables the competing lidar driver and
planner by masking their systemd units. These service choices persist across
reboots even when the vendor boot controller explicitly tries to start them.

## One-time setup

```bash
cd /var/opt/robot/data/dimos-m20-kronknav
./dimos/robot/deeprobotics/m20/deploy/setup.sh
```

The script prompts for the normal GOS `sudo` password and the NOS SSH/`sudo`
password. It is safe to run again.

## Start DimOS

```bash
cd /var/opt/robot/data/dimos-m20-kronknav
uv sync --extra all
source .venv/bin/activate
dimos --build-native --robot-ip 10.21.31.106 --rerun-host 0.0.0.0 \
  run deeprobotics-m20-kronknav-control --daemon
```

`--robot-ip` names the NOS Zenoh bridge. `M20Connection` and the camera relay
still use the documented AOS endpoints at `10.21.31.103`.

Connect the operator viewer directly to the Rerun bridge on GOS:

```bash
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

The Rerun bridge, camera relay, mapping, planning, and control modules all run
inside the GOS control blueprint. The desktop viewer is only a client.

Attach the control shell:

```bash
source .venv/bin/activate
dimos shell
```

```python
app.M20Connection.standup()
app.M20Connection.set_navigation_terrain("stairs")
app.M20Connection.liedown()
```
