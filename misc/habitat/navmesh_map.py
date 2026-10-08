# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Ground-truth navigation map of one Habitat scene, from the navmesh the robot drives on.

Writes one JSON file: the navmesh's top-down view as an embedded PNG (white is
drivable), the pixel-to-world mapping, and a spawn and goal whose drivable route
is a few rooms long. Runs in habitat-sim's own Python 3.9 env:

    target/habitat/env/bin/python misc/habitat/navmesh_map.py 106366410_174226806 \\
        ~/Documents/habitat-sim/data/hssd-hab/hssd-hab.scene_dataset_config.json \\
        dimos/evals/suites/maps/106366410_174226806.navmesh.json \\
        --goal-ros 2.1433 -2.4401 --spawn-clearance-m 1.0 --route-m 6 12 \\
        --min-straight-m 5 --min-detour 1.0
"""

import argparse
import base64
import io
import json

import habitat_sim as hs
import numpy as np
from PIL import Image

# ROS world (x forward, y left, z up) from Habitat (y up, -z forward); see habitat/frames.py.
R_ROS_HAB = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene_id")
    parser.add_argument("scene_dataset_config")
    parser.add_argument("out")
    parser.add_argument("--meters-per-pixel", type=float, default=0.05)
    parser.add_argument("--route-m", type=float, nargs=2, default=(9.0, 14.0))
    parser.add_argument("--min-straight-m", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--spawn-clearance-m", type=float, default=0.0)
    parser.add_argument("--min-detour", type=float, default=1.25)  # route / straight line
    parser.add_argument("--goal-ros", type=float, nargs=2, default=None, metavar=("X", "Y"))
    args = parser.parse_args()

    backend = hs.SimulatorConfiguration()
    backend.scene_dataset_config_file = args.scene_dataset_config
    backend.scene_id = args.scene_id
    backend.enable_physics = False
    agent = hs.agent.AgentConfiguration()
    agent.action_space = {}
    sim = hs.Simulator(hs.Configuration(backend, [agent]))
    pathfinder = sim.pathfinder
    if not pathfinder.is_loaded:  # as the dimos Habitat server builds it
        settings = hs.NavMeshSettings()
        settings.include_static_objects = True
        settings.cell_height = settings.cell_size
        assert sim.recompute_navmesh(pathfinder, settings)

    lo, hi = pathfinder.get_bounds()
    floor = float(lo[1])
    mpp = args.meters_per_pixel
    # Rows run along Habitat +z (ROS -x), columns along Habitat +x (ROS -y), so the image
    # is drawn with ROS +x up and ROS +y left.
    mask = np.asarray(pathfinder.get_topdown_view(mpp, floor + 0.1))
    png = io.BytesIO()
    Image.fromarray((mask * 255).astype(np.uint8)).save(png, format="PNG")

    pathfinder.seed(args.seed)
    rng = np.random.default_rng(args.seed)
    points = [pathfinder.get_random_navigable_point() for _ in range(400)]
    points = [p for p in points if abs(p[1] - floor) < 0.3]
    if args.goal_ros is not None:
        gx, gy = args.goal_ros
        goal = pathfinder.snap_point(np.asarray([-gy, floor, -gx], dtype=np.float32))
        points.append(goal)
    for _ in range(5000):
        i, j = rng.integers(len(points), size=2)
        if args.goal_ros is not None:
            j = len(points) - 1
        if pathfinder.distance_to_closest_obstacle(points[i], 2.0) < args.spawn_clearance_m:
            continue
        path = hs.ShortestPath()
        path.requested_start, path.requested_end = points[i], points[j]
        if not pathfinder.find_path(path) or not np.isfinite(path.geodesic_distance):
            continue
        straight = float(np.linalg.norm(np.asarray(points[i]) - np.asarray(points[j])))
        route = float(path.geodesic_distance)
        lo_m, hi_m = args.route_m
        if (
            lo_m < route < hi_m
            and straight > args.min_straight_m
            and route >= args.min_detour * straight
        ):
            break
    else:
        raise RuntimeError("no spawn and goal pair matched the route limits")

    def ros(p: object) -> list[float]:
        return [round(float(v), 4) for v in R_ROS_HAB @ np.asarray(p, dtype=float)]

    out = {
        "scene_id": args.scene_id,
        "frame_id": "world",
        "meters_per_pixel": mpp,
        # World position of pixel (0, 0)'s corner: the image's top-left is max ROS x, max ROS y.
        "origin_ros_xy": [-float(lo[2]), -float(lo[0])],
        "width_px": int(mask.shape[1]),
        "height_px": int(mask.shape[0]),
        "spawn_ros": ros(points[i]),
        "goal_ros": ros(points[j]),
        "route_m": round(route, 3),
        "straight_m": round(straight, 3),
        "reference_path_ros": [ros(p) for p in path.points],
        "navmesh_png_base64": base64.b64encode(png.getvalue()).decode(),
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(f"{args.scene_id}: route {route:.1f} m, straight {straight:.1f} m -> {args.out}")


if __name__ == "__main__":
    main()
