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

"""PerceiveLoopSkill builds its VL model in the background: construction and
skill discovery never wait for the import, and a build failure surfaces on
`look_out_for`."""

from threading import Event

import pytest

from dimos.perception.experimental import perceive_loop_skill as skill_module
from dimos.perception.experimental.perceive_loop_skill import PerceiveLoopSkill


def test_construction_and_skill_discovery_do_not_wait_for_the_model(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    release = Event()
    model = object()

    def gated_create(name: str) -> object:
        release.wait(timeout=5.0)
        return model

    monkeypatch.setattr(skill_module, "create", gated_create)
    module = PerceiveLoopSkill()
    try:
        assert not module._vl_model.done()
        skills = {s.func_name for s in module.get_skills()}
        assert skills == {"look_out_for", "stop_looking_out"}
        assert not module._vl_model.done()
        release.set()
        assert module._vl_model.result(timeout=5.0) is model
    finally:
        release.set()
        module.stop()


def test_model_build_failure_surfaces_on_look_out_for(monkeypatch: pytest.MonkeyPatch) -> None:
    def failing_create(name: str) -> object:
        raise RuntimeError("no torch")

    monkeypatch.setattr(skill_module, "create", failing_create)
    module = PerceiveLoopSkill()
    try:
        with pytest.raises(RuntimeError, match="no torch"):
            module.look_out_for(["cat"])
        assert module._active_lookout == ()
        assert module._lookout_subscription is None
    finally:
        module.stop()
