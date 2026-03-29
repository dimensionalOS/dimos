# Python API Tests

## Overview

Tests for the `dimos.connect()` Python API (issue #1636). The API provides a minimal
programmatic entry point: `dimos.connect("blueprint-name")` returns a `ModuleCoordinator`
directly — no wrapper classes.

## Running the Automated Tests (no hardware needed)

All automated tests use the `StreamTestModule` — a synthetic robot that produces
fake odometry, camera frames, and lidar data. No physical robot or simulator required.

```bash
cd /home/ubuntu/dimos
source .venv/bin/activate
CI=1 python -m pytest dimos/core/test_api.py dimos/core/test_api_advanced.py \
    dimos/core/test_api_usecases.py dimos/core/test_api_streams.py \
    dimos/core/test_api_extended.py dimos/core/test_api_incremental.py -v
```

**Expected result:** 61 passed, 3 xfailed, 0 failures (~4 minutes)

The 3 xfails are `RemoteOut` lacking `.observable()` — a known framework gap.

## Test Files

| File | Tests | What it covers |
|------|-------|---------------|
| `test_api.py` | 8 | Basic connect/stop/list/module lookup |
| `test_api_advanced.py` | 14 | Concurrency, cross-process, lifecycle, payloads |
| `test_api_usecases.py` | 12 | All 10 use cases from issue #1636 |
| `test_api_streams.py` | 12 | Stream subscribe, multi-sub, timing, compose |
| `test_api_extended.py` | 10 | RxPY pipelines, frame rates, odom rates |
| `test_api_incremental.py` | 9 | Manual coordinator, incremental module deploy |

## Physical Robot Tests (Unitree Go2)

These tests verify the Python API works end-to-end on real hardware. Run them
with the Go2 powered on and connected to the same network.

### Prerequisites

```bash
cd /home/ubuntu/dimos
source .venv/bin/activate
```

### Test 1: Walk Forward

Verify the robot walks forward ~1 meter using the Python API.

```python
import dimos

robot = dimos.connect("unitree-go2")
skills = robot.module("UnitreeSkillContainer")

# Stand up first
skills.execute_sport_command("StandUp")

# Walk forward 1 meter
skills.relative_move(forward=1.0)

# Check odometry changed
import threading, time
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

odom = []
e = threading.Event()
unsub = robot.module("GO2Connection").odom.subscribe(
    lambda p: (odom.append(p), e.set())
)
e.wait(timeout=5)
unsub()

print(f"Current position: {odom[-1].pose.position}")
# Expect x to have increased by ~1.0

robot.stop()
```

**What to verify:** Robot physically moves forward, odometry shows ~1m displacement.

### Test 2: Take a Photo

Capture a camera frame from the robot's front camera.

```python
import dimos
import threading

robot = dimos.connect("unitree-go2")
conn = robot.module("GO2Connection")

frame = [None]
e = threading.Event()
unsub = conn.color_image.subscribe(
    lambda img: (frame.__setitem__(0, img), e.set())
)
e.wait(timeout=10)
unsub()

print(f"Got frame: shape={frame[0].data.shape}, dtype={frame[0].data.dtype}")

# Save to disk
from PIL import Image
img = Image.fromarray(frame[0].data)
img.save("/tmp/go2_photo.jpg")
print("Saved to /tmp/go2_photo.jpg")

robot.stop()
```

**What to verify:** `/tmp/go2_photo.jpg` contains a real camera image from the Go2.

### Test 3: Obstacle Distance (LiDAR)

Read lidar data and compute the nearest obstacle distance.

```python
import dimos
import threading
import numpy as np

robot = dimos.connect("unitree-go2")
conn = robot.module("GO2Connection")

cloud = [None]
e = threading.Event()
unsub = conn.lidar.subscribe(
    lambda pc: (cloud.__setitem__(0, pc), e.set())
)
e.wait(timeout=10)
unsub()

points = cloud[0]._pcd_tensor.point['positions'].numpy()
distances = np.linalg.norm(points, axis=1)
min_dist = float(np.min(distances[distances > 0.1]))  # ignore self-reflections
print(f"Nearest obstacle: {min_dist:.2f}m ({len(points)} points)")

robot.stop()
```

**What to verify:** Reports a reasonable distance (e.g. wall at 2-3m), point count > 100.

### Test 4: Walk a Square

Make the robot walk a 1-meter square using sequential moves and turns.

```python
import dimos
import time

robot = dimos.connect("unitree-go2")
skills = robot.module("UnitreeSkillContainer")

skills.execute_sport_command("StandUp")
time.sleep(1)

# Walk a 1m square
for i in range(4):
    print(f"Side {i+1}/4: forward 1m...")
    skills.relative_move(forward=1.0)
    time.sleep(0.5)
    print(f"Side {i+1}/4: turn 90 degrees...")
    skills.relative_move(yaw=1.5708)  # pi/2 radians
    time.sleep(0.5)

print("Square complete!")
robot.stop()
```

**What to verify:** Robot walks a roughly square path and returns near starting position.

### Test 5: Export Odometry to CSV

Record 5 seconds of odometry data and export to CSV.

```python
import dimos
import threading
import time
import csv

robot = dimos.connect("unitree-go2")
conn = robot.module("GO2Connection")

odom_log = []
unsub = conn.odom.subscribe(
    lambda p: odom_log.append({
        "t": time.time(),
        "x": float(p.pose.position.x),
        "y": float(p.pose.position.y),
        "z": float(p.pose.position.z),
    })
)

print("Recording odometry for 5 seconds...")
time.sleep(5)
unsub()

with open("/tmp/go2_odom.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["t", "x", "y", "z"])
    writer.writeheader()
    writer.writerows(odom_log)

print(f"Exported {len(odom_log)} samples to /tmp/go2_odom.csv")
robot.stop()
```

**What to verify:** CSV file has continuous odometry readings at ~10-50Hz, positions look reasonable.

### Test 6: Skill Chaining (Stand Up -> Move -> Sit Down)

Chain multiple skills in sequence.

```python
import dimos
import time

robot = dimos.connect("unitree-go2")
skills = robot.module("UnitreeSkillContainer")

print("Standing up...")
skills.execute_sport_command("StandUp")
time.sleep(2)

print("Walking forward...")
skills.relative_move(forward=0.5)
time.sleep(1)

print("Sitting down...")
skills.execute_sport_command("StandDown")
time.sleep(2)

print("Done!")
robot.stop()
```

**What to verify:** Robot stands, walks half a meter, then sits back down cleanly.

### Test 7: Discover Available Modules

List what's available on the running robot (no motion required).

```python
import dimos

robot = dimos.connect("unitree-go2")

print("Deployed modules:")
for name in robot.module_names:
    print(f"  - {name}")

# Try accessing each module
for name in robot.module_names:
    proxy = robot.module(name)
    print(f"  {name}: {type(proxy)}")

robot.stop()
```

**What to verify:** Lists modules like `GO2Connection`, `UnitreeSkillContainer`, etc.

## Known Gaps

1. **`RemoteOut` lacks `.observable()` / `.get_next()` / `.hot_latest()`** — only `.subscribe()` works for reading streams. This is the biggest API UX gap.
2. **RPC after `stop()` returns `None` silently** — should ideally raise an exception.
3. **`SkillInfo` has no `description` attribute** — skill docstrings aren't exposed via the API.
4. **`get_by_name()` calls `sys.exit(1)` on unknown names** — should raise `ValueError` for programmatic use.

## Architecture

```
dimos.connect("unitree-go2")
    |
    +-- get_by_name("unitree-go2")    -> Blueprint object
    +-- autoconnect(blueprint)         -> wires LCM transports
    +-- blueprint.build()              -> ModuleCoordinator
                                            |
                                            +-- .module("GO2Connection")     -> ModuleProxy (RPC)
                                            +-- .module("UnitreeSkillContainer") -> ModuleProxy
                                            +-- .module_names                -> list[str]
                                            +-- .stop()                      -> clean shutdown
```

`ModuleProxy.__getattr__` forwards method calls over RPC to the module running
in a worker process. Stream outputs (`Out[T]`) are accessible as attributes
(e.g. `proxy.odom`) and support `.subscribe(callback)`.
