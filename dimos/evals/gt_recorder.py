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

"""GT recorder: the simulator's privileged object poses, recorded for scoring.

Subscribes ``/gt_object_poses`` (MujocoSimModule ``publish_ground_truth``)
over LCM and records it to its own db, separate from the agent-visible
recording — ground truth must never leak into the agent's memory. The eval
runner deploys this per case that declares ``ground_truth=True``; scorers
read it back through ``EvalRunner.gt_store()`` and
:mod:`dimos.evals.predicates`.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.memory.module import Recorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class GTRecorder(Recorder):
    """Records the GT object pose stream to ``db_path``.

    GT poses are already world-frame (frame_id = body name, no tf anchor),
    so deploy with ``poseless_streams=[predicates.GT_STREAM]`` and
    ``record_tf=False``. The stream keeps the port name — the wiring layer
    names topics after it.
    """

    gt_object_poses: In[PoseStamped]
