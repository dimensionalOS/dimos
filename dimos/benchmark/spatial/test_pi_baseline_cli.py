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

import json
from pathlib import Path

import pytest

from dimos.benchmark.spatial.pi_baseline.cli import main
from dimos.benchmark.spatial.test_pi_baseline_config import valid_payload


def test_validate_command_is_explicit_and_side_effect_free(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    config = tmp_path / "config.json"
    config.write_text(json.dumps(valid_payload(auth)), encoding="utf-8")
    assert main(["validate", str(config)]) == 0
    assert "configuration is valid" in capsys.readouterr().out
