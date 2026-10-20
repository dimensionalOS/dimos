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


from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import dreamlabs_dual_openyam


def test_protocol_blueprint_has_no_robot_executor() -> None:
    [policy] = dreamlabs_dual_openyam.blueprints
    assert policy.module is DreamlabsModule
    assert not policy.kwargs
