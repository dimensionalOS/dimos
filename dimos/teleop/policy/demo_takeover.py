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

"""Plot all curves, spatial weights and human/policy/blended bottle-cap motion.

Run: python -m dimos.teleop.policy.demo_takeover --output /tmp/policy-teleop.png
"""

import argparse
from itertools import pairwise
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.policy.blending import blend_twist
from dimos.teleop.policy.demo_sources import bottle_cap_commands, bottle_cap_takeover
from dimos.teleop.policy.takeover import (
    Weights6,
    distance_to_target,
    linear,
    logarithmic,
    power,
    quadratic,
    reference_path_progress,
)


def render_demo(output: Path) -> None:
    figure = plt.figure(figsize=(12, 9), layout="constrained")
    curves = figure.add_subplot(221)
    progress = np.linspace(0, 1, 101)
    for curve in (linear, quadratic, power, logarithmic):
        curves.plot(progress, [curve(float(value)) for value in progress], label=curve.__name__)
    curves.set(xlabel="Pose progress lambda", ylabel="Policy weight alpha", title="Takeover curves")
    curves.legend()

    path = [(0.25, -0.1, 0.25), (0.4, -0.1, 0.35), (0.4, 0.0, 0.4)]
    points = np.concatenate([np.linspace(a, b, 51) for a, b in pairwise(path)])
    poses = [Transform(translation=Vector3(point)) for point in points]
    spatial = figure.add_subplot(222, projection="3d")
    colors = [quadratic(reference_path_progress(pose, path)) for pose in poses]
    scatter = spatial.scatter(*points.T, c=colors, cmap="viridis", vmin=0, vmax=1)
    spatial.set(
        xlabel="World X (m)",
        ylabel="World Y (m)",
        zlabel="World Z (m)",
        title="Reference path: policy weight",
    )
    figure.colorbar(scatter, ax=spatial, label="alpha")

    distance = figure.add_subplot(223)
    distance.plot(
        np.arange(len(poses)),
        [logarithmic(distance_to_target(pose, path[-1], 0.3)) for pose in poses],
    )
    distance.set(
        xlabel="Path sample",
        ylabel="Policy weight alpha",
        title="Distance to target + logarithmic curve",
    )

    motion = figure.add_subplot(224)
    dt, duration = 0.01, 8.0
    for mode in ("human", "policy", "blended"):
        z, yaw = 0.3, 0.0
        heights, angles = [z], [yaw]
        for elapsed in np.arange(0, duration, dt):
            pose = Transform(translation=Vector3(0.4, 0.0, z))
            teleop, policy = bottle_cap_commands(float(elapsed), duration, "world")
            weights: float | Weights6 = bottle_cap_takeover(pose)
            if mode == "human":
                weights = 0.0
            elif mode == "policy":
                weights = 1.0
            command = blend_twist(teleop, policy, weights)
            z += command.linear.z * dt
            yaw += command.angular.z * dt
            heights.append(z)
            angles.append(yaw)
        motion.plot(angles, heights, label=mode)
    motion.set(
        xlabel="World Z rotation (rad)",
        ylabel="World Z height (m)",
        title="Bottle cap: lift and rotate, then return",
    )
    motion.legend()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    render_demo(parser.parse_args().output)


if __name__ == "__main__":
    main()
