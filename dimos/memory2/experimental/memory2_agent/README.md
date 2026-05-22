# Memory2 Agent

What does it do?

Given a memory2 database, this agent introspects all streams to perform modality fussion (when the tools are compatible with the modality)
in order to generate rich/useful temporal-spatial representations, domain-structure validation, and measurement capabilities on spatial data.

How do we achieve this?

1. Provide tools to reason over memory2 types (listing streams, summarizing, getting recent observations)
2. Rendering of occupancy maps, and of placing points on it to have visual reference for coordinates.
3. Temporal indexing. As it can be a consistent method of joining between modalities, regardless of sampling rate and measurement error.
4. Searching for entities - Finding (through CLIP) the entities that exist in an embedded stream, and when they appear.
5. Searching for coordinates - Finding (through raytracing) whether a coordinate (2D) has been observed in any of the images the system captured.
6. Walkthrough tools: Allows the system to follow a heavily downsampled video stream's frames, to understand frame by frame what is happening.
7. Calculations - uses a minimized python REPL for mathematical operations
8. Sizing rooms in the occupancy maps by proposing bounding polygon points, then upsizing them/downsizing them based on visual evidence (images),
as well as out-of-room lidar measurements, out-of-room odom measuremennts, and room intersections in order to iteratively refine the border between rooms.
9. Skills - Any hard algorithms that can be achieved by a composition of the previous skills can be previously defined by text. These algorithms can be applied flexibly by the agent. One could think that skill development is a shortcut to providing the way to get the right answer.

## Install

This experimental agent is not covered by the core dependencies. Install the
`agents` extra, which pulls in LangChain, `langchain-openai`, and
`pydantic-monty` (the sandboxed REPL behind the `calc` tool):

```
pip install -e '.[agents]'
```

Set `OPENAI_API_KEY` (and `GOOGLE_API_KEY` if using a `gemini-*` model). For
Gemini you also need `langchain-google-genai`, which is not in the extra.

## Tools

(15 tools, defined in `tools.py`)

### Meta
- **list_skills** — list named skills the agent can load.
- **load_skill** — return the full procedure for a named skill.
- **calc** — sandboxed Python REPL for arithmetic and bookkeeping state.

### Stream introspection
- **list_streams** — list memory streams + item counts.
- **summary** — one-line summary of a stream (count, time range).
- **recent** — return the n most recent observations from a stream.

### Querying recorded data
- **search_semantic** — CLIP semantic search over an embedded stream.
- **near** — spatial filter: observations whose pose is within `radius` of (x, y).
- **show_image** — fetch the recorded image closest to `ts`, inline.
- **recall_view** — return up to k recorded camera frames matching a memory query.

### Map / spatial
- **show_map** — render the top-down occupancy map with agent pose at a moment.
- **verify_room_partition** — overlay proposed room polygons on the occupancy map.
- **frames_facing** — recorded frames whose viewing cone could contain (x, y).

### Walkthrough (sample frames over a time range)
- **walkthrough_timestamps** — evenly-spaced timestamps at ~step_seconds (step 0.5–3.0, ≤32 stamps, must chunk long ranges).
- **walkthrough** — same as above but returns annotated low-res frames (≤16 frames per call).

## Skills

(7 skills, defined in `skills/*.md`)

- **thinking_about_rooms** — any room-based reasoning: segment the space into verified room polygons first, then operate on them — count, size/bounds, map coordinates, contents, time spent per room (dwell via `odom_in` from `verify_room_partition`), and cross-room comparisons. Built on per-frame pose tracking + polygon tracing + `verify_room_partition`.
- **describe_room** — what's in a given room, answered from frames inside that room (composes `thinking_about_rooms` so the answer isn't biased by semantic-search priming).
- **unexplored_spaces** — where to explore next: identifies frontiers from the orange unpartitioned blobs flagged by `verify_room_partition` that are NOT enclosed by walls.
- **count_unique_things** — how many distinct instances of a kind of thing the robot saw, by localising each candidate to a world (x, y) and merging coincident positions.
- **measure_distance** — measure a distance between two things, two coordinates, or along the robot's path.
- **position_of_thing** — world (x, y) of an object the robot saw, primarily via bearing triangulation from two views.
- **perspective** — questions framed from another entity's viewpoint ("what is to the right of X").
