# Copyright 2026 Dimensional Inc.
"""Fixed backend used by the offline demo and smoke tests."""

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


class FakeGraspProposer:
    checkpoint = "fake://graspgenx-ycb-demo-v1"
    device = "cpu (fake backend)"

    def propose_grasps(self, object_pointcloud):  # type: ignore[no-untyped-def]
        center = object_pointcloud.pointcloud.get_center()
        candidates = [
            GraspCandidate(
                Pose(
                    {
                        "position": [float(center[0]), float(center[1]), float(center[2] + 0.16)],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.91,
            ),
            GraspCandidate(
                Pose(
                    {
                        "position": [
                            float(center[0] + 0.01),
                            float(center[1]),
                            float(center[2] + 0.15),
                        ],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.73,
            ),
            GraspCandidate(
                Pose(
                    {
                        "position": [
                            float(center[0] - 0.01),
                            float(center[1]),
                            float(center[2] + 0.14),
                        ],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.52,
            ),
        ]
        return GraspCandidateArray(Header(float(object_pointcloud.ts), "world"), candidates)
