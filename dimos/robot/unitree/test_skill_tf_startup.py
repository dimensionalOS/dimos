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

"""Robot skills consume transforms published before their first invocation."""

from unittest.mock import Mock

from reactivex.subject import Subject

from dimos.core.stream import Transport
from dimos.core.transport_factory import rpc_backend
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer


def test_start_subscribes_before_the_first_pose_lookup(monkeypatch):
    backend = rpc_backend()
    for method in ("start", "serve_module_rpc", "stop"):
        monkeypatch.setattr(backend, method, Mock())
    messages = Subject()
    transport = Mock(spec=Transport)
    transport.subscribe.side_effect = lambda callback, *_: messages.subscribe(callback).dispose
    module = UnitreeSkillContainer()
    module.set_transport("tf", transport)
    try:
        module.start()
        transform = Transform(
            frame_id="world", child_frame_id="base_link", translation=Vector3(1, 2, 0), ts=17.0
        )
        messages.on_next(TFMessage(transform))
        observed = module.tfbuffer.get("world", "base_link", warn=False)
        assert observed is not None
        assert observed.translation.to_numpy().tolist() == [1.0, 2.0, 0.0]
        assert observed.ts == 17.0
    finally:
        module.stop()
        messages.dispose()
