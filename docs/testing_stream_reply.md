# Sensor Replay & Storage Toolkit

A lightweight framework for **recording, storing, and replaying binary data streams for automated tests**.  It keeps your repository small (data lives in Git LFS) while giving you Python‑first ergonomics for working with RxPY streams, point‑clouds, videos, command logs—anything you can pickle.

---

## 1 At a Glance

| Need                           | One liner                                                     |
| ------------------------------ | ------------------------------------------------------------- |
| **Iterate over every message** | `SensorReplay("raw_odometry_rotate_walk").iterate(print)`     |
| **RxPY stream for piping**     | `SensorReplay("raw_odometry_rotate_walk").stream().pipe(...)` |
| **Throttle replay rate**       | `SensorReplay("raw_odometry_rotate_walk").stream(rate_hz=10)` |
| **Raw path to a blob/dir**     | `path = testData("raw_odometry_rotate_walk")`                 |
| **Store a new stream**         | see [`SensorStorage`](#5-storing-new-streams)                 |

> If the requested blob is missing locally, it is transparently downloaded from Git LFS, extracted to `tests/data/<name>/`, and cached for subsequent runs.

---

## 2 Goals

* **Zero setup for CI & collaborators** – data is fetched on demand.
* **No repo bloat** – binaries live in Git LFS; the working tree stays trim.
* **Symmetric API** – `SensorReplay` ↔︎ `SensorStorage`; same name, different direction.
* **Format agnostic** – replay *anything* you can pickle (protobuf, numpy, JPEG, …).
* **Data type agnostic** – with testData("raw_odometry_rotate_walk") you get a Path object back, can be a raw video file, whole codebase, ML model etc 


---

## 3 Replaying Data

### 3.1 Iterating Messages

```python
from sensor_tools import SensorReplay

# Print every stored Odometry message
SensorReplay(name="raw_odometry_rotate_walk").iterate(print)
```

### 3.2 RxPY Streaming

```python
from rx import operators as ops
from operator import sub, add
from sensor_tools import SensorReplay, Odometry

# Compute total yaw rotation (radians)

total_rad = (
    SensorReplay("raw_odometry_rotate_walk", autocast=Odometry.from_msg)
    .stream()
    .pipe(
        ops.map(lambda odom: odom.rot.z),
        ops.pairwise(),  # [1,2,3,4] -> [[1,2], [2,3], [3,4]]
        ops.starmap(sub),  # [sub(1,2), sub(2,3), sub(3,4)]
        ops.reduce(add),
    )
    .run()
)

assert total_rad == pytest.approx(4.05, abs=0.01)
```

### 3.3 Lidar Mapping Example (200MB blob)

```python
from sensor_tools import SensorReplay, LidarMessage
from mapping import Map

lidar_stream = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
map_ = Map(voxel_size=0.5)

# Blocks until the stream is consumed
map_.consume(lidar_stream.stream()).run()

assert map_.costmap.grid.shape == (404, 276)
```

---

## 4 Low Level Access

If you want complete control, call **`testData(name)`** to get a `Path` to the extracted file or directory — no pickling assumptions:

```python
absolute_path: Path = testData("some_name")
```

Do whatever you like: open a video file, load a model checkpoint, etc.

---

## 5 Storing New Streams

1. **Write a test marked `@pytest.mark.tool`** so CI skips it by default.
2. Use `SensorStorage` to persist the stream into `tests/data/<name>/*.pickle`.

```python
@pytest.mark.tool
def test_store_odometry_stream():
    load_dotenv()

    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")
    robot.standup()

    storage = SensorStorage("raw_odometry_rotate_walk2")
    storage.save_stream(robot.raw_odom_stream())  # ← records until interrupted

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        robot.liedown()
```

### 5.1 Behind the Scenes

* Any new file/dir under `tests/data/` is treated as a **data blob**.
* `./bin/lfs_push` compresses it into `tests/data/.lfs/<name>.tar.gz` *and* uploads it to Git LFS.
* Only the `.lfs/` archive is committed; raw binaries remain `.gitignored`.

---

## 6 Storing Arbitrary Binary Data

Just copy to `tests/data/whatever`
* `./bin/lfs_push` compresses it into `tests/data/.lfs/<name>.tar.gz` *and* uploads it to Git LFS.

---

## 7 Developer Workflow Checklist

1. **Drop new data** into `tests/data/`.
2. Run `./bin/lfs_push` (or let the pre commit hook nag you).
3. Commit the resulting `tests/data/.lfs/<name>.tar.gz`.
4. Open PR — reviewers pull only what they need, when they need it.

### 7.1 Pre commit Setup (optional but recommended)

```sh
sudo apt install pre-commit
pre-commit install   # inside repo root
```

Now each commit checks formatting, linting, *and* whether you forgot to push new blobs:

```
$ echo test > tests/data/foo.txt
$ git add tests/data/foo.txt && git commit -m "demo"
LFS data ......................................................... Failed
✗ New test data detected at /tests/data:
  foo.txt
Either delete or run ./bin/lfs_push
```

---

## 8 Future Work

- A replay rate that mirrors the **original message timestamps** can be implemented downstream (e.g., an RxPY operator)
- Likely this same system should be used for production binary data delivery as well (Models etc)

---

## 9 Existing Examples

* `dimos/robot/unitree_webrtc/type/test_odometry.py`
* `dimos/robot/unitree_webrtc/type/test_map.py`

