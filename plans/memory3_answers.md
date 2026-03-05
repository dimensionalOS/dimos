# Memory2 API Answers

Worked examples against the API defined in `memory3.md`.

## Q1: "Where was I when this log line was added?"

> Pose lookup, correlating to log lines found. Assume log lines have poses associated. Assume there are multiple log lines matching a search.

### Setup

```python
store = SqliteStore("/data/robot.db")
session = store.session()

# TextStream for robot logs — pose auto-filled from TF tree
logs = session.text_stream("logs", payload_type=str,
                           pose_provider=lambda: tf.get_pose("world", "base_link"))

# At runtime, just append text — pose is filled automatically
logs.append("Motor fault on joint 3")
logs.append("Obstacle detected ahead")
logs.append("Motor fault on joint 3")
```

### Single log line lookup

```python
row = logs.query().search_text("motor fault on joint 3").one()
print(f"Robot was at {row.pose} when this log was added (t={row.ts})")
```

`search_text()` uses FTS5 keyword matching. `one()` returns the best match. The pose comes straight from `_meta` — no joins or extra queries needed.

### Multiple matches

```python
rows = logs.query().search_text("motor fault").order_by("ts").fetch()

for row in rows:
    text = logs.load(row.ref)  # load actual log text from _payload
    print(f"t={row.ts} pose={row.pose}: {text}")
```

### Spatial aggregation — "where do motor faults cluster?"

```python
rows = logs.query().search_text("motor fault").fetch()

# Group by proximity (application-level, not part of core API)
from collections import defaultdict
clusters = defaultdict(list)
for row in rows:
    # bucket by 2m grid
    key = (round(row.pose.x / 2) * 2, round(row.pose.y / 2) * 2)
    clusters[key].append(row)

for loc, group in clusters.items():
    print(f"  {len(group)} motor faults near {loc}")
```

### What's exercised

- `TextStream` with FTS index for keyword search
- `search_text()` → FTS5 `MATCH`
- Pose stored at append time, returned in `ObservationRow.pose`
- `load()` to retrieve actual text payload separately from metadata
- `order_by("ts")` for chronological ordering
