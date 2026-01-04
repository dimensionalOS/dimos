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

<!--Result:-->
```
Path: /home/lesh/coding/dimos/data/cafe.jpg
Exists: True
```

## How It Works

<details>
<summary>diagram source</summary>

<details><summary>Pikchr</summary>

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

<!--Result:-->
![output](assets/get_data_flow.svg)

</details>

1. Checks if `data/{name}` already exists locally
2. If missing, pulls the `.tar.gz` archive from Git LFS
3. Decompresses the archive to `data/`
4. Returns the `Path` to the extracted file/directory

## Common Patterns

### Loading Images

```python
from dimos.utils.data import get_data
from dimos.msgs.sensor_msgs import Image

image = Image.from_file(get_data("cafe.jpg"))
print(f"Image shape: {image.data.shape}")
```

<!--Result:-->
```
Image shape: (771, 1024, 3)
```

### Loading Model Checkpoints

```python skip
from dimos.utils.data import get_data

model_dir = get_data("models_mobileclip")
checkpoint = model_dir / "mobileclip2_s0.pt"
```

### Loading Recorded Data for Replay

```python skip
from dimos.utils.data import get_data
from dimos.utils.testing.replay import Replay

data_dir = get_data("unitree_office_walk")
replay = Replay(data_dir)
```

### Loading Point Clouds

```python skip
from dimos.utils.data import get_data
from dimos.mapping.pointclouds import read_pointcloud

pointcloud = read_pointcloud(get_data("apartment") / "sum.ply")
```

## Data Directory Structure

Data files live in `data/` at the repo root. Large files are stored in `data/.lfs/` as `.tar.gz` archives tracked by Git LFS.

```
data/
├── cafe.jpg              # Small files: committed directly
├── apartment/            # Directories: extracted from LFS
│   └── sum.ply
└── .lfs/
    └── apartment.tar.gz  # LFS-tracked archive
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

A pre-commit hook ([`bin/lfs_check`](/bin/lfs_check#L26)) blocks commits if you have uncompressed directories in `data/` without a corresponding `.tar.gz` in `data/.lfs/`.

## Location Resolution

When running from:
- **Git repo**: Uses `{repo}/data/`
- **Installed package**: Clones repo to user data dir:
  - Linux: `~/.local/share/dimos/repo/data/`
  - macOS: `~/Library/Application Support/dimos/repo/data/`
  - Fallback: `/tmp/dimos/repo/data/`
