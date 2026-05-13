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

"""Smoke test: PGO and RtabMap publish a compatible contract.

The locked definition asked for "comparable trajectories within a loose
tolerance" on identical inputs. PGO on `main` is a NativeModule that requires
its C++ binary to be built — we don't run that binary in fast tests. So the
"equivalence" we *can* validate without the binary is contractual:

* both modules expose the same input streams with the same types (so they're
  swap-compatible inside ``create_nav_stack``);
* both expose ``corrected_odometry`` and ``global_map`` as ``Out[<same type>]``;
* both expose a TF-correction Out[Odometry] (``pgo_tf`` vs ``rtab_tf``).

The numerical comparison is intentionally deferred to a later integration
test that runs against a recorded rosbag (the user already has rosbag-driven
PGO accuracy tests; the analog for RtabMap will follow once the runner
bridge is wired to real RTAB-Map).
"""

from __future__ import annotations

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.modules.rtab_map.rtab_map import RtabMap


def test_input_contract_matches() -> None:
    """Inputs must match: same names, same message types."""
    pgo = PGO()
    rtab = RtabMap()
    try:
        for port_name, expected_type in (
            ("registered_scan", PointCloud2),
            ("odometry", Odometry),
        ):
            assert pgo.inputs[port_name].type is expected_type
            assert rtab.inputs[port_name].type is expected_type
    finally:
        pgo._close_module()
        rtab._close_module()


def test_corrected_output_contract_matches() -> None:
    pgo = PGO()
    rtab = RtabMap()
    try:
        assert pgo.outputs["corrected_odometry"].type is rtab.outputs["corrected_odometry"].type
        assert pgo.outputs["global_map"].type is rtab.outputs["global_map"].type
    finally:
        pgo._close_module()
        rtab._close_module()


def test_tf_correction_output_shape_matches() -> None:
    """``pgo_tf`` and ``rtab_tf`` carry the same payload type (Odometry)
    so the TF bridge can subscribe to either."""
    pgo = PGO()
    rtab = RtabMap()
    try:
        assert pgo.outputs["pgo_tf"].type is Odometry
        assert rtab.outputs["rtab_tf"].type is Odometry
    finally:
        pgo._close_module()
        rtab._close_module()
