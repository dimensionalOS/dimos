# Localize-backed VQA families

Additive VQA families that consume Dan `Localization` outputs without changing
Dan localize, existing image/point-cloud primitives, or existing image-path
families.

Evidence adapter: `dimos/evals/vqa/primitives/localize_scene.py`  
Sector policy: [`ego-spatial-sectors.md`](ego-spatial-sectors.md)  
Usage: `docs/usage/vqa.md` (`--with-localize`)

---

## Constraints (do not violate)

| Do not | Why |
| --- | --- |
| Change existing image-path families | Keep current GT contracts stable |
| Change existing image/LiDAR primitives | Same |
| Change Dan localize | Treat as frozen sensor API |
| Add near-duplicates of current families | Avoid `verified_presence` ≈ `presence`, etc. |

---

## What image-path families already cover

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

## Shipped families (this branch)

### `ego_fore_aft`

- **Ask:** Is the object ahead of the robot or behind it?
- **Evidence:** Shared ego cones — ahead `|θ| ≤ 45°`, behind `|θ| ≥ 135°`,
  `0.5 ≤ r ≤ 5` m; sides rejected.
- **Referents:** bare unique name, `nearest X`, `X next to Y`.
- **Distinct from:** `object_distance` (magnitude to camera).

### `ego_turn`

- **Ask:** To face this object, turn left / right / already ahead / turn around?
- **Evidence:** Same cones/wedges; behind → **turn around** (not rejected).
- **Distinct from:** `horizontal_direction` (image u-coordinate).

### `ego_any_fore_aft`

- **Ask:** Is any verified instance of category X in the ahead cone?
- **Evidence:** Existential over category hits in the ahead cone only.

### `height_order`

- **Ask:** Which of two objects is higher?
- **Evidence:** World-`z` with a minimum vertical margin; unique categories only.
- **Distinct from:** No image-path family uses world height.

### Multi-instance + coverage

- Proposals: unique / nearest / next-to / any-ahead (no `left`/`right` `<category>`).
- Optional CLI: `--with-localize`, `--localize-query`, `--localize-pad`,
  `--localize-coverage-k` via `localize_recording.py`.
- Coverage: ≤K published localize cases **per frame** (default K=5), round-robin
  `(family, selector)` and within-cell category balance (`localize_coverage.py`).

---

## Backlog (not shipped)

| Family | Notes |
| --- | --- |
| `pairwise_relation` | A vs B in robot heading frame |
| `extent_larger_3d` | Point-cloud AABB extent vs `largest_visible_area` |
| `flank_same_side` | Same / opposite lateral sides |
| `seen_order` | Category encounter order in a window (weak without re-id) |

---

## Out of scope without more stack

Attribute, object state, functional reasoning, world knowledge, true
episodic re-id — need extra models or memory, not frozen localize alone.
