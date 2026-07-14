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

from collections.abc import Iterator
import json

import pytest

from dimos.skills.mapping.floorplan_skill import FloorplanSkillContainer


@pytest.fixture
def container() -> Iterator[FloorplanSkillContainer]:
    module = FloorplanSkillContainer()
    yield module
    module.stop()


def test_skill_registers_with_schema(container: FloorplanSkillContainer) -> None:
    skills = container.get_skills()
    names = [s.func_name for s in skills]
    assert "generate_floorplan" in names

    info = next(s for s in skills if s.func_name == "generate_floorplan")
    schema = json.loads(info.args_schema)
    text = json.dumps(schema).lower()
    # the schema is the tool description the agent sees — it must sell the
    # analysis use and document + type every parameter
    assert "attribute" in text or "analy" in text
    params = schema.get("properties", schema)
    for param in (
        "rrd_path",
        "duration",
        "explore",
        "session_dir",
        "levels",
        "ai_review",
        "ai_render_styles",
        "include_furniture",
        "save_3d_model",
        "debug_artifacts",
        "output_prefix",
        "project_name",
    ):
        assert param in json.dumps(params), f"{param} missing from schema"
    # debug output and 3D-model writing must be OFF unless the caller asks
    props = schema.get("properties", {})
    if "save_3d_model" in props:
        assert props["save_3d_model"].get("default") in (False, None)
        assert props["debug_artifacts"].get("default") in (False, None)
        # ...but stylized AI renditions are ON by default (all three styles),
        # so they're attached as images the calling agent can see unprompted.
        assert props["ai_render_styles"].get("default") == "drafted,cyanotype,presentation"


def test_skill_fails_gracefully_on_missing_recording(
    container: FloorplanSkillContainer,
) -> None:
    result = container.generate_floorplan(rrd_path="/nonexistent/never.rrd")
    assert isinstance(result, str)
    assert "failed" in result.lower()
