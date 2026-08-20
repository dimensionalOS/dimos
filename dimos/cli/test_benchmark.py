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

from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.protocol.pubsub.benchmark.model import Environment, Stack


def test_transport_benchmark_cli_runs_filtered_campaign(mocker, tmp_path) -> None:
    run_campaign = mocker.patch(
        "dimos.protocol.pubsub.benchmark.runner.run_campaign",
        return_value=tmp_path,
    )

    result = CliRunner().invoke(
        main,
        [
            "benchmark",
            "transport",
            "run",
            "--suite",
            "smoke",
            "--stack",
            "lcm",
            "--environment",
            "local",
            "--output",
            str(tmp_path),
        ],
    )

    assert result.exit_code == 0
    assert str(tmp_path) in result.stdout
    assert run_campaign.call_args.kwargs["stacks"] == {Stack.LCM}
    assert run_campaign.call_args.kwargs["environments"] == {Environment.LOCAL}


def test_transport_benchmark_cli_rejects_unknown_suite(tmp_path) -> None:
    result = CliRunner().invoke(
        main,
        [
            "benchmark",
            "transport",
            "run",
            "--suite",
            "unknown",
            "--output",
            str(tmp_path),
        ],
    )

    assert result.exit_code == 2
    assert "smoke" in result.output
