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

"""Add where ICP says the source keyframe belongs, and the source-to-there gap before/after, to the example jsons."""

import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

# usage: <script> <run dir> <examples.json> [<run dir> <examples.json> ...]
for run, path in zip(map(Path, sys.argv[1::2]), map(Path, sys.argv[2::2]), strict=False):
    name = path.stem.removeprefix("loop_examples_")
    kf = np.fromfile(run / "a.kf", rm.KEYFRAME)
    t0 = kf["ts"][0]
    states = []
    for ts, rec in rm.read_closures(run / "a.kf.closures"):
        dq = R.from_quat(rec["dq"].astype(float))
        states.append(
            (
                ts,
                list(rec["id"].astype(int)),
                dq.apply(rec["xyz"].astype(float)) + rec["dt"],
                dq * R.from_quat(rec["q"].astype(float)),
            )
        )
    state_ts = np.array([s[0] for s in states])

    def at(ts, states=states, state_ts=state_ts):
        return states[int(np.searchsorted(state_ts, ts, side="right")) - 1]

    data = json.loads(path.read_text())
    proposed = {
        (d["source_id"], d["target_id"]): d["source_time"]
        for d in (lambda j: j.get("closure_diagnostics", j.get("loop_diagnostics")))(
            json.loads((run / "a.json").read_text())
        )
    }
    for e in data["examples"]:
        a, b = e["source_id"], e["target_id"]
        e["path_step"] = max(1, (a + 1) // 900)
        _, ids, xyz, rot = at(proposed[(a, b)])
        if a not in ids or b not in ids:
            print(name, a, b, "source not in the graph when proposed")
            continue
        xa, xb, rb = xyz[ids.index(a)], xyz[ids.index(b)], rot[ids.index(b)]
        assert np.allclose(xa[:2], e["source_xy_before"], atol=0.01), (xa, e["source_xy_before"])
        implied = np.array(e["icp_rotation"]) @ xa + np.array(e["icp_translation"])
        e["icp_source_xy"] = np.round(implied[:2], 3).tolist()
        e["gap_before_m"] = round(float(np.linalg.norm(implied - xa)), 3)
        # ICP's placement held fixed relative to the target keyframe, so it rides along when the graph moves
        implied_in_target = rb.inv().apply(implied - xb)
        if e["applied_s"] is not None:
            _, ids2, xyz2, rot2 = at(e["applied_s"] + t0 + 1e-6)
        if e["applied_s"] is not None and a in ids2 and b in ids2:
            xa2, xb2, rb2 = xyz2[ids2.index(a)], xyz2[ids2.index(b)], rot2[ids2.index(b)]
            implied2 = rb2.apply(implied_in_target) + xb2
            e["icp_source_xy_after"] = np.round(implied2[:2], 3).tolist()
            e["gap_after_m"] = round(float(np.linalg.norm(implied2 - xa2)), 3)
        print(
            name,
            a,
            b,
            "yank",
            round(e["inputs"]["yank_translation_m"], 3),
            "gap",
            e["gap_before_m"],
            "->",
            e.get("gap_after_m"),
        )
    path.write_text(json.dumps(data))
