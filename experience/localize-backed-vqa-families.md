# Localize-backed VQA families (proposed)

Proposed **new** VQA families that consume Dan's `Localization` outputs
without changing:

- Dan's perception / localize implementation
- Existing VQA primitives (`edge_tam`, `range`, …)
- Existing VQA families (`presence`, `horizontal_direction`, `object_count`,
  `image_coverage`, `largest_visible_area`, `object_distance`, `closest_object`)

Evidence path: add a thin `LocalizeEvidenceAdapter` (new file only) that reads
`list[Localization]` plus robot/camera pose from the recording window.

Related: OpenEQA spatial / situated categories; DimOS localize at
`dimos/perception/localize/`.

---

## Constraints (do not violate)

| Do not | Why |
| --- | --- |
| Change existing families | Keep current GT contracts stable |
| Change existing primitives | Same |
| Change Dan localize | Treat as frozen sensor API |
| Add near-duplicates of current families | Avoid `verified_presence` ≈ `presence`, `ego_distance` ≈ `object_distance`, `closest_to_robot` ≈ `closest_object`, window `count` ≈ `object_count` |

---

## What existing families already cover

| Family | Semantic niche |
| --- | --- |
| `presence` | Object category visible in **this image** |
| `object_count` | Instance count in **this image** |
| `horizontal_direction` | Object in **image** left / center / right |
| `image_coverage` | **Pixel** coverage of a mask |
| `largest_visible_area` | Largest **2D mask** among candidates |
| `object_distance` | Range to **camera** (LiDAR-in-mask buckets) |
| `closest_object` | Closest to **camera** among candidates |

---

## Proposed new families

### 1. `ego_turn`

- **Ask:** To face this object, should the robot turn left, right, or is it
  already ahead?
- **Example:** To face the door, should I turn left, right, or am I already
  facing it?
- **Evidence:** Instance `position_world_xyz` vs `base_link` yaw; bearing in
  robot frame → `{left, right, ahead}` (ahead = within a fixed angle gate).
- **Distinct from:** `horizontal_direction` (image u-coordinate, not body yaw).

### 2. `ego_fore_aft`

- **Ask:** Is the object ahead of the robot or behind it?
- **Example:** Is the chair ahead of me or behind me?
- **Evidence:** Sign of the instance position projected on the robot forward
  axis.
- **Distinct from:** `object_distance` (magnitude to camera, not fore/aft).

### 3. `pairwise_relation`

- **Ask:** Relative pose of object A vs object B in the robot heading frame
  (or world frame with an explicit “from my heading” clause).
- **Example:** Is the plant to the left of the table (from my heading)?
- **Evidence:** Two verified instances; vector B−A (or A vs B) expressed in
  robot yaw frame → `{left, right, ahead, behind}` (pick a closed choice set).
- **Distinct from:** Single-object `horizontal_direction`; not camera-range
  `closest_object`.

### 4. `height_order`

- **Ask:** Which of two objects is higher?
- **Example:** Which is higher, the shelf or the table?
- **Evidence:** Compare `xyz.z` (or point-cloud z percentile) with a minimum
  vertical margin; reject if margin too small.
- **Distinct from:** No current family uses world height.

### 5. `extent_larger_3d`

- **Ask:** Which object is larger in 3D extent?
- **Example:** Which object is larger in 3D extent, the sofa or the table?
- **Evidence:** Point-cloud AABB diagonal or max extent; require margin (e.g.
  20%) like `largest_visible_area`.
- **Distinct from:** `largest_visible_area` (2D mask pixels, not meters).

### 6. `flank_same_side`

- **Ask:** Are two objects on the same lateral side of the robot, or opposite
  sides?
- **Example:** Are the two chairs on the same side of me, or opposite sides?
- **Evidence:** Sign of lateral (robot-y) coordinate for each instance;
  `{same_side, opposite_sides}`; reject if either is near the midline.
- **Distinct from:** No pairwise same/opposite family today.

### 7. `seen_order` (weak episodic)

- **Ask:** In this walk / window pair, was category A encountered before
  category B?
- **Example:** In this walk, did I encounter a door before a table?
- **Evidence:** Compare first- or last-seen timestamps of verified instances
  per category across one episode or two ordered windows. **No re-id** — order
  is by category, not “the same bag”.
- **Distinct from:** All current families are single-frame.
- **Caveat:** Weakest GT story of this list; needs a clear episode protocol.

---

## Implementation sketch (additive only)

```text
dimos/evals/vqa/
  primitives/localize_evidence.py   # NEW adapter only
  families.py                       # append FamilySpec + answer_* only
  contracts.py                      # extend FamilyName Literal only
  test_contracts.py                 # new tests only
docs/usage/vqa.md                   # document new rows
```

Do not edit existing `answer_presence` / `answer_object_distance` / …
Do not edit `primitives/edge_tam.py`, `primitives/range.py`, …

Offline path: run `tool_localize` (or in-process `localize`) on a window →
feed `list[Localization]` into the adapter → `answer_question` for the new
family only.

---

## Suggested implementation order

Ranked by **low ambiguity first**, then **implementation difficulty**.

| Priority | Family | Ambiguity risk | Impl. difficulty | Why this order |
| --- | --- | --- | --- | --- |
| 1 | `ego_fore_aft` | Low (binary sign on forward axis; reject near 0) | Easy | One instance + pose; clear ahead/behind |
| 2 | `height_order` | Low if vertical margin enforced | Easy | Pure z compare; little pose math |
| 3 | `ego_turn` | Medium (need ahead-cone + left/right split) | Easy–medium | Same geometry as fore_aft + angle buckets |
| 4 | `extent_larger_3d` | Medium (sparse lidar extent noisy) | Medium | Need cloud stats + margin reject |
| 5 | `flank_same_side` | Medium (midline rejects; need two solid hits) | Medium | Two laterals + midline gate |
| 6 | `pairwise_relation` | Higher (4-way relation + heading frame) | Medium–hard | Most choice collisions; tight rejects |
| 7 | `seen_order` | Highest without re-id | Medium protocol, weak GT | Needs episode contract; category-only order |

**First slice to ship:** `ego_fore_aft` + `height_order` (+ optional `ego_turn`).
Three families, one adapter, strong distinction from the existing seven.

### Implemented (this branch)

- Adapter: `dimos/evals/vqa/primitives/localize_scene.py`
- Families: `ego_fore_aft`, `height_order`, `ego_turn` (additive only)
- Author helper: `FixedQuestionAuthor` for fixture / offline proposals
- Generation: `GenerationFrame.localize_scene` + `requires_localize` filter
- Optional CLI: `--with-localize` + `--localize-query` (default off) via
  `dimos/evals/vqa/localize_recording.py`
- Coverage budget: `dimos/evals/vqa/localize_coverage.py` keeps ≤K published cases per
  `(family, selector)` (default K=5 with `--with-localize`; `--localize-coverage-k 0` disables)

---

## Out of scope without more stack

Attribute, object state, functional reasoning, world knowledge, true
episodic re-id — need extra models or memory, not frozen localize alone.
