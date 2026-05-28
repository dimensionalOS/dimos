"""Plant the MuJoCo sim person at a fixed spot for the greeter demo.

The office sim injects a ``person`` mocap body (``jeong_seun_34`` mesh, added by
``dimos.simulation.mujoco.model._add_person_object``) that
``PersonPositionController`` drives from the ``/person_pose`` LCM topic. The
person-follow demo walks it along a track; the greeter instead wants it standing
still where the dog can find it.

This broadcasts a fixed pose, repeatedly, so the person stays put until you stop
it (Ctrl-C). The simulation must already be running to receive the pose.

    # terminal 1: the greeter sim (see run_greeter_sim.sh)
    # terminal 2: plant the person ~2.5 m ahead, facing back toward the dog
    PYTHONPATH=src python scripts/sim_person_pose.py --x 2.5 --y 0.0
"""

from __future__ import annotations

import argparse
import math
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plant the simulation person at a fixed pose."
    )
    parser.add_argument("--x", type=float, default=2.5, help="World x (m).")
    parser.add_argument("--y", type=float, default=0.0, help="World y (m).")
    parser.add_argument(
        "--yaw", type=float, default=math.pi,
        help="Heading (rad); default faces back toward the spawn origin.",
    )
    parser.add_argument(
        "--rate-hz", type=float, default=2.0,
        help="Re-broadcast rate so the pose persists across the sim loop.",
    )
    args = parser.parse_args()

    half_yaw = args.yaw / 2.0
    pose = Pose(
        position=[args.x, args.y, 0.0],
        orientation=[0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)],
    )
    period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.5
    transport: LCMTransport[Pose] = LCMTransport("/person_pose", Pose)
    print(
        f"Planting sim person at ({args.x}, {args.y}), yaw {args.yaw:.2f} rad. "
        "Ctrl-C to stop."
    )
    try:
        while True:
            transport.broadcast(None, pose)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        transport.stop()


if __name__ == "__main__":
    main()
