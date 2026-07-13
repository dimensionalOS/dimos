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

from pathlib import Path
import time

import pytest
from pytest_mock import MockerFixture

import demo_benchmark_python as benchmark
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.mapping.costmapper import CostMapper as PythonCostMapper
from dimos.mapping.native_costmapper.module import CostMapper as RustCostMapper


class _OtherModule(Module):
    pass


def _result(
    implementation: str,
    *,
    replay_db: str = "go2_bigoffice",
) -> benchmark.SystemBenchmarkResult:
    return benchmark.SystemBenchmarkResult(
        label=implementation.title(),
        implementation=implementation,
        blueprint="unitree-go2",
        replay_db=replay_db,
        transport="lcm",
        requested_duration_seconds=60.0,
        observed_duration_seconds=60.1,
        sample_interval_seconds=1.0,
        interrupted=False,
        samples=[
            benchmark.ResourceSample(1.0, 10.0, 100.0, 12),
            benchmark.ResourceSample(2.0, 20.0, 120.0, 13),
            benchmark.ResourceSample(3.0, 30.0, 140.0, 13),
        ],
    )


def test_parser_defaults_to_full_go2_replay(tmp_path: Path) -> None:
    args = benchmark._build_parser().parse_args(["--out", str(tmp_path)])

    assert args.implementation == "python"
    assert args.blueprint == "unitree-go2"
    assert args.replay_db == "go2_bigoffice"
    assert args.duration == pytest.approx(60.0)


def test_rust_blueprint_only_swaps_costmapper(mocker: MockerFixture) -> None:
    source = autoconnect(
        _OtherModule.blueprint(),
        PythonCostMapper.blueprint(),
    ).global_config(n_workers=9)
    mocker.patch.object(benchmark, "get_blueprint_by_name", return_value=source)

    selected = benchmark._select_blueprint("unitree-go2", "rust")
    active_modules = [atom.module for atom in selected.active_blueprints]

    assert active_modules == [_OtherModule, RustCostMapper]
    assert PythonCostMapper not in active_modules
    assert selected.global_config_overrides["n_workers"] == 9


def test_result_round_trip_preserves_whole_system_summary() -> None:
    original = _result("python")

    restored = benchmark.SystemBenchmarkResult.from_dict(original.to_dict())

    assert restored == original
    assert restored.median_cpu_percent == pytest.approx(20.0)
    assert restored.p95_cpu_percent == pytest.approx(30.0)
    assert restored.median_rss_mib == pytest.approx(120.0)
    assert restored.peak_rss_mib == pytest.approx(140.0)


def test_dtop_sampler_aggregates_all_live_process_cpu(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    sampler = benchmark.DtopAggregateSampler(tmp_path / "host.jsonl")
    sampler._started_at = time.monotonic()
    mocker.patch.object(sampler, "_tree_rss", return_value=(512.0, 14))

    sampler._record_sample(
        {
            "coordinator": {"cpu_percent": 7.0},
            "workers": [
                {"alive": True, "cpu_percent": 12.0},
                {"alive": True, "cpu_percent": 5.0},
                {"alive": False, "cpu_percent": 99.0},
            ],
        }
    )

    assert len(sampler.samples) == 1
    assert sampler.samples[0].cpu_percent == pytest.approx(24.0)
    assert sampler.samples[0].rss_mib == pytest.approx(512.0)
    assert sampler.samples[0].process_count == 14
    assert (tmp_path / "host.jsonl").read_text(encoding="utf-8").count("\n") == 1


def test_comparison_rejects_different_workloads() -> None:
    with pytest.raises(ValueError, match="same blueprint, replay"):
        benchmark._validate_comparison([_result("python"), _result("rust", replay_db="different")])
