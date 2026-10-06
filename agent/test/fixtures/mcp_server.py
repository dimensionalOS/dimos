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

"""Actual DimOS MCP handlers with deterministic tools; no coordinator or robot."""

import json
import os

import numpy as np
import uvicorn

from dimos.agents.mcp.mcp_server import app
from dimos.agents.skill_result import SkillResult
from dimos.core.module import SkillInfo
from dimos.msgs.sensor_msgs.Image import Image

app.state.skills = [
    SkillInfo(
        class_name="Fixture",
        func_name="echo",
        args_schema=json.dumps(
            {"type": "object", "properties": {"value": {"type": "string"}}, "required": ["value"]}
        ),
    )
]
for name in ("image", "failure"):
    app.state.skills.append(
        SkillInfo(
            class_name="Fixture",
            func_name=name,
            args_schema=json.dumps({"type": "object", "properties": {}}),
        )
    )
app.state.rpc_calls = {
    "echo": lambda value, **kwargs: value,
    "image": lambda **kwargs: Image(np.zeros((8, 8, 3), dtype=np.uint8)),
    "failure": lambda **kwargs: SkillResult.fail("FIXTURE_FAILURE", "Expected failure"),
}
uvicorn.run(app, host="127.0.0.1", port=int(os.environ["DIMCODE_TEST_PORT"]), log_level="error")
