# Data Loading

The [`get_data`](/dimos/utils/data.py) function provides access to test data and model files, handling Git LFS downloads automatically.

## Basic Usage

```python
from dimos.utils.data import get_data

# Get path to a data file/directory
data_path = get_data("cafe.jpg")
print(f"Path: {data_path}")
print(f"Exists: {data_path.exists()}")
```

```results
Path: /home/lesh/coding/dimos/data/cafe.jpg
Exists: True
```

## How It Works

<details>
<summary>Pikchr</summary>

```pikchr fold output=assets/get_data_flow.svg
color = white
fill = none

A: box "get_data(name)" rad 5px fit wid 170% ht 170%
arrow right 0.4in
B: box "Check" "data/{name}" rad 5px fit wid 170% ht 170%

# Branch: exists
arrow from B.e right 0.3in then up 0.4in then right 0.3in
C: box "Return path" rad 5px fit wid 170% ht 170%

# Branch: missing
arrow from B.e right 0.3in then down 0.4in then right 0.3in
D: box "Pull LFS" rad 5px fit wid 170% ht 170%
arrow right 0.3in
E: box "Decompress" rad 5px fit wid 170% ht 170%
arrow right 0.3in
F: box "Return path" rad 5px fit wid 170% ht 170%
```

</details>

![output](assets/get_data_flow.svg)

1. Checks if `data/{name}` already exists locally
2. If missing, pulls the `.tar.gz` archive from Git LFS
3. Decompresses the archive to `data/`
4. Returns the `Path` to the extracted file/directory

## Common Patterns

### Loading Images

```python
from dimos.utils.data import get_data
from dimos.msgs.sensor_msgs.Image import Image

image = Image.from_file(get_data("cafe.jpg"))
print(f"Image shape: {image.data.shape}")
```

```results
Image shape: (771, 1024, 3)
```

### Loading Model Checkpoints

```python skip
from dimos.utils.data import get_data

model_dir = get_data("models_edgetam")
checkpoint = model_dir / "edgetam.pt"
print(f"Checkpoint: {checkpoint.name} ({checkpoint.stat().st_size // 1024}KB)")
```

```results
Checkpoint: edgetam.pt (54801KB)
```

### Loading Recorded Data for Replay

```python skip
from dimos.utils.data import get_data
from dimos.utils.testing.legacy_pickle import LegacyPickleStore

data_dir = get_data("unitree_office_walk")
replay = LegacyPickleStore(data_dir / "lidar")
print(f"Replay {type(replay).__name__} loaded from: {data_dir.name}")
print(replay.find_closest_seek(1))
```

```results
Replay LegacyPickleStore loaded from: unitree_office_walk
{'type': 'msg', 'topic': 'rt/utlidar/voxel_map_compressed', 'data': {'stamp': 1751591000.0, 'frame_id': 'odom', 'resolution': 0.05, 'src_size': 77824, 'origin': [-3.625, -3.275, -0.575], 'width': [128, 128, 38], 'data': {'points': array([[ 2.725, -1.025, -0.575],
       [ 2.525, -0.275, -0.575],
       [ 2.575, -0.275, -0.575],
       ...,
       [ 2.675, -0.525,  0.775],
       [ 2.375,  1.175,  0.775],
       [ 2.325,  1.225,  0.775]], shape=(22730, 3))}}}
```

### Loading Point Clouds

```python skip
from dimos.utils.data import get_data
from dimos.mapping.pointclouds.util import read_pointcloud

pointcloud = read_pointcloud(get_data("apartment") / "sum.ply")
print(f"Loaded pointcloud with {len(pointcloud.points)} points")
```

```results
Loaded pointcloud with 63672 points
```

## Data Directory Structure

Data files live in `data/` at the repo root. Large files are stored in `data/.lfs/` as `.tar.gz` archives tracked by Git LFS.

<details>
<summary>Diagram</summary>

```diagon fold mode=Tree
data/
  cafe.jpg
  apartment/
    sum.ply
  .lfs/
    cafe.jpg.tar.gz
    apartment.tar.gz
```

</details>

```results
data/
 ├──cafe.jpg
 ├──apartment/
 │   └──sum.ply
 └──.lfs/
     ├──cafe.jpg.tar.gz
     └──apartment.tar.gz
```


## Adding New Data

### Small Files (< 1MB)

Commit directly to `data/`:

```sh skip
cp my_image.jpg data/

# 2. Compress and upload to LFS
./bin/lfs_push

git add data/.lfs/my_image.jpg.tar.gz

git commit -m "Add test image"
```

### Large Files or Directories

Use the LFS workflow:

```sh skip
# 1. Copy data to data/
cp -r my_dataset/ data/

# 2. Compress and upload to LFS
./bin/lfs_push

git add data/.lfs/my_dataset.tar.gz

# 3. Commit the .tar.gz reference
git commit -m "Add my_dataset test data"
```

The [`lfs_push`](/bin/lfs_push) script:
1. Compresses `data/my_dataset/` → `data/.lfs/my_dataset.tar.gz`
2. Uploads to Git LFS
3. Stages the compressed file

A pre-commit hook ([`bin/hooks/lfs_check`](/bin/hooks/lfs_check#L26)) blocks commits if you have uncompressed directories in `data/` without a corresponding `.tar.gz` in `data/.lfs/`.

## Location Resolution

When running from:
- **Git repo**: Uses `{repo}/data/`
- **Installed package**: Clones repo to user data dir:
  - Linux: `~/.local/share/dimos/repo/data/`
  - macOS: `~/Library/Application Support/dimos/repo/data/`
  - Fallback: `/tmp/dimos/repo/data/`
- **`DIMOS_DATA_DIR` set**: Uses that directory and nothing else. It takes precedence over both cases above.

### A Data Directory Without a Repository

Set `DIMOS_DATA_DIR` to the unpacked assets shipped with an installation. dimOS can look up those assets offline. Individual blueprints can have additional dependencies. For example, Alfred obtains its OpenArm description from a separate Git asset cache.

```sh skip
DIMOS_DATA_DIR=/opt/dimos/data dimos --simulation run unitree-go2
```

The directory holds what `data/` holds after extraction: `cafe.jpg`, `mujoco_sim/`, and so on. The `.tar.gz` archives are not needed.

`get_data(name)` returns `$DIMOS_DATA_DIR/{name}`. A name that is not there raises `FileNotFoundError` with the name and the directory. Nothing is cloned, pulled or extracted.

The directory can be mounted read-only. Close SQLite recordings before packaging them so committed transactions are checkpointed into the database. Replay opens packaged recordings as immutable snapshots without creating journal files.

With this setting, generated scene packages, benchmark exports, and named legacy pickle recordings go under `$XDG_STATE_HOME/dimos` (normally `~/.local/state/dimos`). A named legacy recording or benchmark export in the state directory takes precedence when reopened by its consumer. Keep explicit output paths outside the packaged directory.
