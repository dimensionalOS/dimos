# tars-sdk

SDK for TARS, the four-slab robot from *Interstellar*. It exposes a high-level velocity API over a MuJoCo simulation. It has no dimos dependency, so it can be published to PyPI on its own. The dimos integration is `dimos/hardware/drive_trains/tars` (twist-base adapter) and `dimos/robot/tars/blueprints.py`.

```bash
uv pip install -e experimental/tars_sdk          # into the dimos venv
```

```python
from tars_sdk import TarsClient

c = TarsClient()              # realtime=False → deterministic, step with c.step(seconds)
# TarsClient(scale=1.0)                                  # film size (default 0.5: 0.76 m, 7.5 kg)
# TarsClient(scene="path/scene.xml", spawn=(x, y, yaw))  # drop TARS into any MuJoCo scene
# TarsClient(viewer=True)                                # MuJoCo window (separate process)
c.connect()                   # loads the model, starts the sim thread
c.stand()
c.move(0.15, 0.1)             # vx m/s, wz rad/s; resend within cmd_timeout (0.5 s)
c.set_mode("roll")            # slabs become wheel spokes (~1 m/s); set_mode("walk") folds back
s = c.get_state()             # mode, odom (estimated), odom_gt (sim truth), raw sensors
frame = c.get_camera()        # RGB + depth (m) from the front camera on slab 2
c.sit(); c.disconnect()
```

Keyboard teleop in the viewer: `mjpython -m tars_sdk.teleop [--scene X.xml] [--scale 1.0] [--roll]` (macOS; `python` on Linux). Arrows drive, R toggles walk/roll.

## Layout

| Module | Role |
|---|---|
| `client.py` | `TarsClient`: public API, command watchdog |
| `gait.py` | Pair gait: (vx, wz) → per-joint PD + feedforward targets |
| `roll.py` | Roll mode: spokes locked 90° apart, telescoping to a virtual rim |
| `estimator.py` | Leg odometry: encoders + foot contact + IMU → pose/velocity |
| `assist.py` | Sim-only hub wrench (see below) |
| `sim.py` | MuJoCo backend: physics, joint PD servos, sensors, camera |
| `kinematics.py` | Slab IK/FK with rolling flat feet |
| `scaling.py` | Froude scaling: every tunable is written for the 1.52 m robot and scaled to `Params.scale` |
| `mirror.py` | Viewer process that mirrors a running sim through shared memory |
| `model/` | `params.py` (every dimension, mass, limit), `generate.py` (MJCF, optional URDF/STL), `tars.xml` |

Model (reference size; the default build is `scale=0.5`, i.e. 0.76 m and 7.5 kg): 1.52 m tall and 60 kg. It has 4 slabs, each with a hinge (±π, 600 Nm) at the hub axle and a telescoping lower segment (0–0.7 m, 800 N). Foot friction is 1.2. Sensors are an IMU on the hub, a touch sensor per foot, and the front camera. To change the design, edit `model/params.py`. The simulator builds the MJCF from `Params` at connect time; `python -m tars_sdk.model.generate [--urdf out/]` writes `tars.xml` (and URDF/STL) to disk.

## Gait

The outer pair (slabs 1 and 4) and the inner pair (2 and 3) alternate as two legs. Each cycle is swing A → shift → swing B → shift. During a swing, the planted pair stands vertical on flat feet. During a shift, all four feet are down and the hub moves over the pair that just landed. The step length is fixed at 0.6 m, because steps under about 0.45 m leave the centre of mass outside the support. Speed is set by cadence instead: up to about 0.4 m/s actual (0.71 commanded).

Turning ("twist") happens while the hub is over the inner pair. The planted outer pair pushes its left and right feet in opposite directions, and the feet skid.

## Roll mode

`set_mode("roll")` waits until the robot is standing, then folds the slabs out into four spokes 90° apart, alternating left and right so ground contacts switch sides. Each planted spoke telescopes to `h / cos(angle)`, which keeps the hub level over a virtual rim. The next spoke pre-extends to land, and the trailing spoke lets go only once the next one carries load. `set_mode("walk")` brakes, folds every spoke straight down, and stands up.

Why the design changes were needed:
- **0.7 m slide travel**: holding the hub level to a 45° hand-off takes 0.57 m.
- **600 Nm hinges**: keep the wheel from splaying.
- **Foot friction 1.2**: the two planted spokes at a hand-off form a 90° A-frame, which needs μ ≥ 1.

Rolling over one planted spoke at constant height is an inverted pendulum. Crossing a hand-off on momentum alone would need about 3.7 m/s, so the assist supplies it (see below).

## Sim assist (on by default)

The stock design (pitch-only hinges, flat feet, one shared axle) **can't walk quasi-statically**:
- Every leg can only push the hub *away* from its own contact point. Nothing can move the hub forward over the trailing pair until the centre of mass is already past that pair's toe.
- Hinge torques that would turn all the slabs the same way just spin the hub.
- Yaw requires the feet to slip.

`assist.py` supplies the missing pieces with a bounded virtual wrench on the hub: a tilt PD (≤300 Nm), a forward push along the gait plan (≤250 N), and heading tracking (≤400 Nm). In roll mode it corrects only sideways lean, and the push cancels the inverted-pendulum pull (weight × foot offset / height) plus a speed term (≤700 N). Everything else is real physics. Pass `TarsClient(assist=None)` to work on unassisted locomotion.

The ways to remove the assist are ankle pitch joints (which give feet that stay flat while the slab tilts), or a dynamic, RL-trained gait.

## Performance (assisted, flat ground)

- Half size (default): about 0.53 m/s top speed, with odometry drift around 2%. Film size: about 0.66 m/s (odometry drift about 11%). The push assist provides up to about 1.3× body weight, so speed is largely assisted.
- Leg-odometry drift is about 1–2% on straight lines. It's worse in turns because the feet skid.
- Turning in place tracks the commanded wz, up to 0.3 rad/s.
- Roll mode averages about 1.1 m/s at 1.2 commanded (about 0.6 at 0.6), in reverse and in arcs. Leg odometry drifts about 10% while rolling.
- In non-realtime mode the sim runs about 80× real time.

Tests: `pytest experimental/tars_sdk/tests -o addopts=""`.
