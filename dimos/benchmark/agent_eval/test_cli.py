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

import pytest

from dimos.benchmark.agent_eval.cli import _parse_endpoint, main


def test_parse_dimsim_endpoint() -> None:
    assert _parse_endpoint("http://127.0.0.1:8090") == ("127.0.0.1", 8090)
    with pytest.raises(ValueError):
        _parse_endpoint("http://127.0.0.1")
    with pytest.raises(ValueError):
        _parse_endpoint("http://127.0.0.1:99999")


def test_cli_preflight_failure_has_stable_nonzero_exit(tmp_path, capsys) -> None:
    code = main(["run", "--config", str(tmp_path / "missing.json")])

    assert code == 2
    assert "preflight failed" in capsys.readouterr().err
