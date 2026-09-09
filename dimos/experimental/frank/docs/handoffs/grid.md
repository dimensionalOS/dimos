# Handoff: FRANK spatial awareness (`grid.py`)

FRANK is a Unitree Go2 robot dog driven by an LLM agent on a laptop through shell scripts in
`dimos/experimental/frank/`. The agent can already move (`robot.py move x y`), look
(`robot.py observe`), and speak. What it lacks is any idea of the space around it: where the walls
are, which way is open, how far it can go, whether a spot is reachable. You are building that.

    dimos/experimental/frank/tools/grid.py

Read `SKILL.md` and `robot.py` in that directory first, then `TASKS.md`, and tick your line when done.

## Where the map comes from

The running DimOS Go2 stack publishes a 2D occupancy grid on the `global_costmap` stream
(`dimos/mapping/costmapper.py`, type `dimos/msgs/nav_msgs/OccupancyGrid.py`) built from lidar, and
the robot pose on `odom`. `robot.py` shows how to attach to the running instance:
`Dimos.connect()` then `app.peek_stream("global_costmap")` / `peek_stream("odom")`. World frame is
the one `move x y` uses: +x east, +y north, metres. Read the OccupancyGrid class for cell values,
resolution, origin, and any helpers it already has; do not reimplement what it provides.

Existing reference for style and for rendering: `.claude/skills/lidar-scene-reading/scene.py`
(renders point clouds to PNG for the agent to Read). Your renders should look similar so the agent
can relate them.

## What to build

One script, importable and CLI. An LLM is the user, so every command prints a short, plain answer.

```bash
uv run python dimos/experimental/frank/tools/grid.py render [out.png]         # plan view: free/unknown/occupied, robot as an arrow, 1 m grid, axes labelled in world metres
uv run python dimos/experimental/frank/tools/grid.py at 2.0 -1.5              # "free" | "occupied" | "unknown" | "outside map"  at world x y
uv run python dimos/experimental/frank/tools/grid.py ray 0                    # how far the robot can go in that heading (deg, 0 = +x world; --relative makes 0 = straight ahead) before hitting occupied/unknown; prints metres and what stopped it
uv run python dimos/experimental/frank/tools/grid.py look                     # eight rays around the robot: "ahead 3.2 m (wall), ahead-left 0.8 m (unknown), ..."
uv run python dimos/experimental/frank/tools/grid.py nearest-free 2.0 -1.5    # closest free cell to a world point, with clearance, e.g. for "stand 1.5 m in front of a person"
uv run python dimos/experimental/frank/tools/grid.py reachable 2.0 -1.5       # is there a free-cell path from the robot to there (BFS on the grid); prints yes/no and rough path length
uv run python dimos/experimental/frank/tools/grid.py open-spots [--n 3]       # a few large open areas (centre, radius) sorted by distance, for "go somewhere with room"
uv run python dimos/experimental/frank/tools/grid.py front 2.0 -1.5 90 1.5    # the world point 1.5 m in front of a pose (x y yaw_deg); for approaching a person head-on
```

Rules for the answers:
- Distances in metres to one decimal, headings in degrees, always say which frame.
- Treat unknown as not traversable for `ray`, `reachable`, and `open-spots`, but report it as
  "unknown" rather than "wall" so the agent can decide to look.
- Inflate obstacles by the robot's half-width (about 0.35 m) for `reachable`, `nearest-free`, and
  `open-spots`. Say so in the output once, not every line.
- `render` must be readable at a glance: robot arrow, north-up, a scale, occupied dark, unknown grey,
  free white, and the eight `look` rays drawn faintly. Keep the image under about 1200 px.
- Importable API mirroring the CLI: `load()`, `at(x, y)`, `ray(heading_deg, relative=False)`,
  `look()`, `nearest_free(x, y)`, `reachable(x, y)`, `open_spots(n)`, `front(x, y, yaw_deg, d)`,
  `render(path)`.

Then add a "Space" section to `SKILL.md` (under 15 lines, match the existing tone) telling the agent
when to use these: render once when arriving somewhere new, `look` before any move, `front` +
`nearest-free` + `reachable` to pick a spot to approach a person, and never move into unknown.

## Testing without the robot

Run `uv run dimos status` first. If an instance is running, use it read-only and do not launch
anything. If nothing is running, the replay stack publishes a real costmap from recorded lidar:

```bash
uv run dimos --transport lcm --replay run unitree-go2 --daemon     # --transport lcm is required on this machine
uv run dimos status
...
uv run dimos stop
```

Give it a minute to build a map, then exercise every command. Read your own render and check it
against `ray`/`look` numbers by eye. Stop the instance when you're done. If replay won't start,
say what happened rather than fighting it for long; a unit test against a hand-built OccupancyGrid
(`test_grid.py` next to the script) is the fallback and is welcome either way.

## Constraints

- No new dependencies. numpy, Pillow, matplotlib are in the venv.
- Do not touch anything outside `dimos/experimental/frank/`. Do not edit `dimos/`. Do not commit.
- Never send motion commands. You only read streams.
- Write for humans: short functions, plain names.

## Hand back

What works, what you tested it against (replay or synthetic), a sample `look` output and a render,
and anything about the costmap that surprised you (resolution, update rate, what unknown looks like).
