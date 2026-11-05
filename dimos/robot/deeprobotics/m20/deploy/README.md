# M20 deployment

Run setup and the complete DimOS stack, including the Rerun bridge, on GOS
(`10.21.31.104`). Setup builds and installs the NOS bridge, runs the RoboSense
driver beside vendor LIO on NOS, and disables the competing lidar driver and
planner by masking their systemd units. These service choices persist across
reboots even when the vendor boot controller explicitly tries to start them.

## Layout on GOS

GOS runs Ubuntu 20.04 (glibc 2.31) and has no internet on the robot hotspot, so
everything is built off the robot and copied over:

- `/var/opt/robot/data/dimos-m20`: the checkout, its `.venv`, and the native
  modules prebuilt for aarch64 and glibc 2.31.
- `/var/opt/robot/data/m20-deps`: Python 3.12, `uv`, the wheelhouse, zenoh-c and
  the `dimos-lcm` headers.

A laptop on the robot hotspot reaches GOS through AOS:

```bash
sudo ip route add 10.21.31.0/24 via 10.21.41.1
```

## One-time setup

From the laptop, with agent forwarding so GOS can reach NOS with the laptop's key:

```bash
ssh -A user@10.21.31.104 'cd /var/opt/robot/data/dimos-m20 && source .venv/bin/activate && ./dimos/robot/deeprobotics/m20/deploy/setup.sh'
```

The script prompts for any `sudo` or NOS SSH password it needs. It is safe to
run again.

## Start DimOS

The robot stands up when `deeprobotics-m20-kronknav-control` starts and sits down when it stops, so clear
the space around it first.

```bash
cd /var/opt/robot/data/dimos-m20
source .venv/bin/activate
dimos --robot-ip 10.21.31.106 \
  run deeprobotics-m20-kronknav-control --rerunbridgemodule.host 0.0.0.0 \
  --rerunwebsocketserver.host 0.0.0.0 --daemon
```

`--robot-ip` names the NOS Zenoh bridge. `M20Connection` and the camera relay
still use the documented AOS endpoints at `10.21.31.103`.

Follow the log with `dimos log -f`. `dimos stop` sits the robot down and stops
DimOS.

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
