# Visual Question Answering

The VQA tools generate deterministic questions from recorded images and evaluate them through the
shared dimOS evaluation runner.

## Generate a Dataset

Generate from one image in a recording containing `color_image`, `camera_info`, `tf`, and either
`pointlio_lidar` or `lidar`:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db --image-index 100
```

Generate from a range. `stop` is exclusive:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db --start 0 --stop 100 --stride 10
```

Or keep update frequency with mem `QualityWindow` on `Image.sharpness`
(same path as loop-closure / semantic-peak picking). Optional
`--min-brightness` drops dark frames first. Do not combine with `--stride`:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db \
  --start 0 --stop 100 --quality-window 0.5 --min-brightness 0.1
```

Generated datasets default to:

```text
~/.local/state/dimos/datasets/vqa/<recording-stem>-frames
```

Use `--output <directory>` to override that location. The destination must be empty. Generation
requires non-empty `color_image`, `camera_info`, `tf`, and either `pointlio_lidar` or `lidar` streams;
incomplete datasets are rejected at startup. For a valid dataset, point-cloud evidence is available
when image and LiDAR observations are within `0.1` seconds. Individual frames with unmatched LiDAR
observations or unresolvable TF retain only image-based question families.

### Point-cloud frame preparation

Point-cloud question families use an explicit image-aligned frame boundary. After validating the
required dataset streams, the loader in `dimos/evals/vqa/pointcloud_frame.py` reads the selected image
and nearest point cloud, rectifies the image and camera intrinsics, resolves the
point-cloud-to-camera transform from recorded TF, and packages the synchronized observations as a
`PointCloudFrame`.

The range primitive in `dimos/evals/vqa/primitives/range.py` consumes that prepared frame. It
projects valid points into the image, keeps the nearest camera-depth point per pixel, selects points
inside each object mask, and derives robust range statistics. Keeping dataset access and calibration
preparation outside the primitive lets range estimation operate only on explicit geometry.

## Question Families

The image-only question author selects object names and applicable families. It does not produce
answers. Every proposal uses `object_names`; single-object families require one entry, while
`closest_object` requires two to five.

| Family | Choices | Evidence rule |
|---|---|---|
| `presence` | yes, no | At least one Moondream detection |
| `horizontal_direction` | left, center, right | Exactly one detection; use its horizontal center |
| `object_count` | one, two, three, four or more | Count Moondream detections |
| `image_coverage` | Adaptive percentage buckets | Divide EdgeTAM mask pixels by image pixels |
| `largest_visible_area` | Two to five authored object references | Select a mask at least 20% larger than the runner-up |
| `object_distance` | under 1 m, 1 to under 2 m, 2 to under 3 m, 3 m or more | Median LiDAR range inside an EdgeTAM mask |
| `closest_object` | Two to five authored object references | Select the smallest unambiguous LiDAR range |
| `ego_fore_aft` | ahead, behind | Shared ±45° / ±135° ego cones for a referent: bare unique name, `nearest X`, or `X next to Y` |
| `height_order` | Two authored object references | Higher world-`z` with a minimum vertical margin (localize scene) |
| `ego_turn` | left, right, ahead, turn around | Bearing wedges in the robot yaw frame (ahead cone + turn around behind) |
| `ego_any_fore_aft` | yes, no | Whether any verified instance of a category lies in the ahead cone |

Proposals without sufficient evidence are rejected and retained in the private audit. If no proposal
can be answered, generation fails without publishing a dataset.

Distance questions require one Moondream detection, one EdgeTAM mask, and at least five projected
LiDAR points. Evidence whose range quartiles cross an answer boundary is rejected.
Closest-object questions accept references such as `left person`, `right person`, and `chair`, which
lets Moondream distinguish repeated categories. They are rejected unless every reference has exactly
one detection and the closest range interval does not overlap any other candidate.

Image-coverage questions choose between two bucket schemes so the measured mask coverage is as far
as possible from the nearest answer boundary. Largest-visible-area questions are rejected unless the
winning EdgeTAM mask contains at least 20% more pixels than the runner-up. Both families work without
point-cloud data.

Localize-backed families (`ego_fore_aft`, `height_order`, `ego_turn`, `ego_any_fore_aft`,
`cross_view_same_object`) are **off by default**. Enable them with `--with-localize`.
Generation then:

1. Discovers object names in a padded time window with prompt-free YOLO-E (or uses optional
   `--localize-query` overrides)
2. Runs Dan localize **once** with a room-scale Go2 policy (not the xArm desktop defaults) to
   verify those names in 3D, build first-sighting timestamps, collect multi-view sightings per
   instance, and cache window `color_image` frames
3. Resolves `base_link` pose per selected image and attaches `LocalizeSceneEvidence`
4. Adds deterministic localize proposals for case diversity:
   - **unique** bare names when a category has exactly one verified hit
   - **nearest** `<category>` when multiple hits exist (closest to the robot)
   - **`<anchor> next to <landmark>`** relational referents for `ego_fore_aft` / `ego_turn`
     when a unique nearby pair exists (disambiguates without leaking turn direction)
   - **any-ahead** yes/no for each selected category (`ego_any_fore_aft`)
   - **height_order** pairs among unique categories only

Ego geometry uses shared cones (`θ = atan2(left, forward)`, `0.5 ≤ r ≤ 5` m): ahead
`|θ| ≤ 45°`, behind `|θ| ≥ 135°`, sides rejected for fore/aft; `ego_turn` answers
left / right / ahead / **turn around**.

These families are **pose-grounded for labels** (verified 3D + `base_link`) but
**memory-clip for the agent**: each ego case publishes a short dense `color_image`
clip from the referent's **first sighting** through the **question time**
(``query − first_spot ≤ 8`` s), and the question text states that relative
`t=0.0s … t=duration` span. Ego cases never fall back to a single RGB frame.

Publish balances two bands (~1:1 when both exist):

- **Hard memory**: `ego_fore_aft=behind`, or `ego_turn∈{left,right,turn around}`
  (bearing outside the 45° ahead cone; on Go2 ~77° HFOV this is usually out of RGB).
- **Soft / in-view**: `ahead`, `ego_any` yes/no, `height_order`, etc.

The default `QuestionAnswer` agent encodes every frame in the clip
(length ≤ `frames_per_stream`).

Left/right ``<category>`` referents are not proposed (awkward English in the question text;
multi-instance disambiguation uses nearest / next-to instead).
`nearest` rejects ties closer than 0.35 m so the answer does not depend on hit order.

Published localize cases are budgeted to at most **K=5 per frame**. Selection
round-robins `(family, selector)` types across frames so every frame with ego
candidates keeps at least one localize question and later frames are not starved.
Within the target type, under-represented categories win. Override with
`--localize-coverage-k`, or pass `0` to publish every answered localize case.
Image-path families are never budgeted.

After ego clips, the same localize evidence materializes **cross-view identity** pairs
(`cross_view_same_object`): propose same/different instance pairs from multi-view
sightings (≥ `--min-views` distinct poses), then cut two short dense clips from the
cached `color_frames` at each sighting timestamp — **no second localize / `--query` pass**.

Each `--with-localize` run also writes a lightweight
`audit/localize_evidence.json` (object name, instance id, world xyz, first_seen,
sighting timestamps + bbox — no RGB/point clouds). Reuse it to skip Dan:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db \
  --start 180 --stop 280 --stride 20 \
  --with-localize \
  --localize-evidence /path/to/prior-dataset/audit/localize_evidence.json
```

Window/`dataset` must match the dump (pad + selected frames → same `[after, before]`).
Color frames are still read from the recording at those timestamps.

Without `--with-localize`, those families are not offered.

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db \
  --start 180 --stop 280 --stride 20 \
  --with-localize \
  --localize-pad 20 \
  --localize-coverage-k 5 \
  --min-views 3
```

## Dataset Layout

```text
dataset/
  cases.jsonl
  labels.jsonl
  assets/
    frame-000100.png          # single-frame families (presence, count, …)
    clips/<case-id>/…         # memory-clip families (ego, cross-view)
  audit/
    run.json
    localize_evidence.json  # lightweight hits for --localize-evidence reuse
    frame-000100/
      frame.json
      ground_truth.json
      cases.json
      labels.json
```

Each public case uses exactly one modality: `image`, `clip`, or `clips`.

- **Single-frame** (`image`): image-only families. Evaluated with `ImageFile`.
- **One short clip** (`clip`): ego-spatial memory. The clip spans **first sighting of the
  referent through the question time**; the question text states the relative
  `t=0.0s … t=duration` window. Evaluated with `ImageClip` → MemoryStore `clip` stream.
- **Two short clips** (`clips`): cross-view identity. Each sighting gets a short dense
  clip (fully encoded; not a long video sparsely subsampled to 8 frames). Streams are
  named `first_clip` / `second_clip`; the question names each span.

`cases.jsonl` and `assets/` are public evaluation inputs. `labels.jsonl` and `audit/` contain private
answers, evidence, rejected proposals, source indices, and timestamps.

## Run Evaluation

```bash skip
dimos evals vqa run ~/.local/state/dimos/datasets/vqa/calibrated-recording-frames \
  --model gpt-4o-mini
```

The default agent (`QuestionAnswer`) encodes every frame in each clip stream when the clip
length is at most `frames_per_stream` (default 8). Generation keeps clips at or under that
budget and always includes required anchors (first spot + query for ego; both sightings for
cross-view). Results land under `~/.local/state/dimos/evals/run-*/`.
