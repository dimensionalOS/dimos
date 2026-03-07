# Query Objects — 4D Region + Soft Scoring System

## Problem

We need to query observations across 4 dimensions (x, y, z, t) plus embedding space. Current API has flat `filter_*` methods — works for simple cases but doesn't compose. We need:

1. **Regions** — composable hard boundaries (include/exclude)
2. **Fields** — soft scoring that biases toward a point/time/embedding without hard cutoffs
3. A way to combine both in a single query

## Key Insight

Hard filters and soft biases are the same thing at different extremes:
- Hard filter = step function (1 inside, 0 outside)
- Soft bias = smooth decay (gaussian, linear, etc.)

A unified **Criterion** type handles both. Each criterion maps an observation to a score in `[0, 1]`. Hard filters are just criteria with score `{0, 1}`.

## Primitives

### Temporal

```python
# Hard boundaries
TimeRange(t1, t2)                  # 1 inside, 0 outside
Before(t)                          # sugar for TimeRange(-inf, t)
After(t)                           # sugar for TimeRange(t, inf)

# Soft — score decays with distance from target
TimeProximity(target, sigma=60.0)  # gaussian: exp(-dt²/2σ²)
```

### Spatial

```python
# Hard boundaries
Sphere(center: PoseLike, radius: float)    # 1 inside, 0 outside
Box(min: PoseLike, max: PoseLike)          # axis-aligned bounding box
HeightRange(z_min, z_max)                  # horizontal slice

# Soft
SpatialProximity(point: PoseLike, sigma=5.0)  # gaussian in 3D
```

### Embedding

```python
# Soft only (no hard boundary in embedding space makes sense)
EmbeddingSimilarity(vector, candidate_k=100)  # cosine similarity, top-k pre-filter
```

### Tags

```python
TagMatch(robot_id="robot1")        # hard: exact match on tag values
```

## Composition

Criteria compose via set operators:

```python
# Intersection — all criteria must score > 0
region = TimeRange(t1, t2) & Sphere(point, 5.0)

# Union — any criterion scoring > 0 passes
region = Sphere(p1, 3.0) | Sphere(p2, 3.0)

# Complement
region = ~TimeRange(t1, t2)  # everything outside this window
```

For soft criteria, composition combines scores:
- `a & b` → `min(a.score, b.score)` (conservative)
- `a | b` → `max(a.score, b.score)` (permissive)

## Weighted Scoring

The interesting problem: "I care about embedding similarity, temporal proximity, AND spatial proximity" — but as soft preferences, not hard cutoffs.

```python
Score(
    time=TimeProximity(target_t, sigma=60),
    space=SpatialProximity(point, sigma=5.0),
    embedding=EmbeddingSimilarity(vector, candidate_k=200),
    weights={"time": 0.3, "space": 0.3, "embedding": 0.4}
)
```

Each dimension produces a `[0, 1]` score. Final score = weighted sum. This replaces the vague `rank(**weights)` in the current API.

## Integration with Query

```python
# Current flat API (still works, sugar for simple cases)
q.after(t).near(point, 5.0).search_embedding(vec, candidate_k=100)

# Region object approach
region = After(t) & Sphere(point, 5.0)
q.where(region).search_embedding(vec, candidate_k=100)

# Full soft scoring — no hard boundaries, just preferences
q.score(
    time=TimeProximity(target_t, sigma=120),
    space=SpatialProximity(point, sigma=10.0),
    embedding=EmbeddingSimilarity(vec, candidate_k=500),
).limit(20)

# Mixed — hard boundary + soft ranking within
q.where(TimeRange(t1, t2)).score(
    space=SpatialProximity(point, sigma=5.0),
    embedding=EmbeddingSimilarity(vec, candidate_k=200),
).limit(10)
```

## SQL Mapping (SQLite impl)

How each primitive maps to SQL:

| Criterion                | SQL Strategy                                          |
|--------------------------|-------------------------------------------------------|
| `TimeRange(t1, t2)`      | `WHERE ts BETWEEN ? AND ?` (B-tree)                   |
| `Before(t)` / `After(t)` | `WHERE ts < ?` / `WHERE ts > ?`                       |
| `Sphere(p, r)`           | R*Tree range query on `_rtree`                        |
| `HeightRange(z1, z2)`    | `WHERE pose_z BETWEEN ? AND ?`                        |
| `Box(min, max)`          | R*Tree range query                                    |
| `TimeProximity(t, σ)`    | `ORDER BY ABS(ts - ?) ASC` or compute score in SELECT |
| `SpatialProximity(p, σ)` | R*Tree range (pre-filter at ~3σ) + score in SELECT    |
| `EmbeddingSimilarity`    | sqlite-vec `MATCH` → temp table                       |
| `TagMatch`               | `WHERE json_extract(tags, ?) = ?`                     |

Soft scoring strategy: **generous hard pre-filter in SQL, then score in Python**.
- Each soft criterion auto-generates a hard pre-filter at ~3σ (captures 99.7% of relevant results)
- `TimeProximity(t, σ=60)` → SQL: `WHERE ts BETWEEN t-180 AND t+180` (B-tree)
- `SpatialProximity(p, σ=5)` → SQL: R*Tree range query with 15m box
- `EmbeddingSimilarity` → sqlite-vec `MATCH` top-k (already a pre-filter)
- Python computes `[0, 1]` scores on the pre-filtered set, applies weights, sorts

This keeps SQL simple (range queries on indexes) and Python handles the math.

## Open Questions

2. **How does `Score` interact with `search_embedding`?** Embedding search already returns ranked results from vec0. Should `Score.embedding` just re-weight those scores, or does it need a separate search pass?

3. **Region objects as first-class types?** Do we store/serialize regions (e.g., "the kitchen region" as a reusable spatial boundary)? Or are they always constructed in code?

4. **Do we need `NOT` regions for exclusion zones?** E.g., "everywhere except within 2m of the charging station."  `~Sphere(charger, 2.0)` — complement on spatial regions requires scanning all of `_meta`, can't use R*Tree efficiently.

5. **Gradient fields?** "Prefer observations taken at higher elevation" — not proximity to a point but a directional preference. `HeightGradient(ascending=True)` as a scorer?

## Priority

- **Phase 1**: Keep the flat `filter_*` / `rank()` API. Implement primitives internally.
- **Phase 2**: Expose `Criterion` objects + `where()` + `score()` as the composable API.
- **Phase 3**: Region persistence, named regions, gradient fields.
