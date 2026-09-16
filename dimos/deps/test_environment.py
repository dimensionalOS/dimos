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

from packaging.requirements import Requirement

from dimos.deps.catalog import Plan
from dimos.deps.environment import (
    EnvironmentReport,
    Outcome,
    RequirementIssue,
    check_environment,
    check_native,
    check_packages,
    check_system,
    direct_requirements,
    expand_extras,
    find_conflicts,
    format_report,
)

REQUIRES = [
    "numpy>=1.26.4",
    "open3d>=0.18.0; platform_system != 'Linux' or platform_machine != 'aarch64'",
    "open3d-unofficial-arm>=0.19; platform_system == 'Linux' and platform_machine == 'aarch64'",
    'langchain>=1.2.3,<2; extra == "agents"',
    'fastapi>=0.115.6; extra == "web"',
    'transformers[torch]>=4.53.0,<4.54; extra == "perception"',
    'dimos[agents,web,perception]; extra == "base"',
    'dimos[base,mapping]; extra == "unitree"',
    'unitree-webrtc-connect>=2.1.2; extra == "unitree"',
    "gtsam-extended>=4.3a1.post1; python_full_version >= '3.11' and extra == \"mapping\"",
    "onnxruntime-gpu>=1.17.1; platform_machine == 'x86_64' and extra == \"cuda\"",
]
LINUX = {
    "platform_system": "Linux",
    "platform_machine": "x86_64",
    "sys_platform": "linux",
    "python_full_version": "3.12.11",
    "python_version": "3.12",
}
ARM = {
    **LINUX,
    "platform_machine": "aarch64",
    "python_full_version": "3.10.14",
    "python_version": "3.10",
}
VERSIONS = {
    "dimos": "0.0.14",
    "numpy": "2.2.6",
    "open3d": "0.19.0",
    "fastapi": "0.115.6",
    "transformers": "4.53.3",
    "torch": "2.7.1+cu128",
    "onnxruntime": "1.24.1",
    "onnxruntime-gpu": "1.24.1",
}
NESTED = {
    "transformers": [
        'torch>=2.1; extra == "torch"',
        'accelerate>=0.26.0; extra == "torch"',
        "numpy",
    ]
}


def _names(active: list[tuple[Requirement, str | None]]) -> dict[str, str | None]:
    return {requirement.name: via for requirement, via in active}


def test_expand_extras_follows_self_references() -> None:
    assert expand_extras(REQUIRES, ["unitree"], LINUX) == {
        "unitree",
        "base",
        "mapping",
        "agents",
        "web",
        "perception",
    }
    assert expand_extras(REQUIRES, [], LINUX) == frozenset()


def test_direct_requirements_core_only() -> None:
    active, excluded = direct_requirements(REQUIRES, [], LINUX)
    assert _names(active) == {"numpy": None, "open3d": None}
    assert excluded == []
    active, _excluded = direct_requirements(REQUIRES, [], ARM)
    assert _names(active) == {"numpy": None, "open3d-unofficial-arm": None}


def test_direct_requirements_with_extras_and_markers() -> None:
    active, excluded = direct_requirements(REQUIRES, ["web", "cuda"], LINUX)
    assert _names(active) == {
        "numpy": None,
        "open3d": None,
        "fastapi": "web",
        "onnxruntime-gpu": "cuda",
    }
    assert excluded == []
    active, excluded = direct_requirements(REQUIRES, ["cuda", "mapping"], ARM)
    assert "onnxruntime-gpu" not in _names(active)
    assert excluded == [
        'gtsam-extended>=4.3a1.post1 (python_full_version >= "3.11" and extra == "mapping")',
        'onnxruntime-gpu>=1.17.1 (platform_machine == "x86_64" and extra == "cuda")',
    ]


def test_check_packages_missing_mismatch_and_nested_extras() -> None:
    active, _ = direct_requirements(REQUIRES, ["unitree"], LINUX)
    missing, mismatched = check_packages(
        active, VERSIONS, requires_of=lambda name: NESTED.get(name), environment=LINUX
    )
    assert {issue.requirement for issue in missing} == {
        "langchain<2,>=1.2.3",
        "unitree-webrtc-connect>=2.1.2",
        "gtsam-extended>=4.3a1.post1",
        "accelerate>=0.26.0",
    }
    accelerate = next(issue for issue in missing if issue.requirement.startswith("accelerate"))
    assert accelerate.via_extra == "transformers[torch]"
    assert mismatched == []
    old = {**VERSIONS, "numpy": "1.21.0", "transformers": "4.60.0.dev0"}
    _missing, mismatched = check_packages(
        active, old, requires_of=lambda name: NESTED.get(name), environment=LINUX
    )
    assert {issue.requirement for issue in mismatched} == {
        "numpy>=1.26.4",
        "transformers[torch]<4.54,>=4.53.0",
    }


def test_conflicts_and_report_flags() -> None:
    assert find_conflicts(VERSIONS) == [("onnxruntime", "onnxruntime-gpu")]
    plan = Plan(extras=frozenset({"web"}), tools=frozenset({"python3"}))
    report = check_environment(plan, environment=LINUX, requires=REQUIRES, versions=VERSIONS)
    assert report.satisfied_for_launch and report.ok
    assert report.conflicts == [("onnxruntime", "onnxruntime-gpu")]
    assert report.tools["python3"].status == "satisfied"
    plan = Plan(extras=frozenset({"web"}), tools=frozenset({"no-such-tool-xyz"}))
    report = check_environment(plan, environment=LINUX, requires=REQUIRES, versions=VERSIONS)
    assert report.satisfied_for_launch and not report.ok
    assert report.tools == {"no-such-tool-xyz": Outcome("missing", "not on PATH")}
    report = check_environment(
        Plan(extras=frozenset({"agents"})), environment=LINUX, requires=REQUIRES, versions=VERSIONS
    )
    assert not report.satisfied_for_launch and not report.ok
    assert [issue.reason for issue in report.missing] == ["missing"]
    report = check_environment(Plan(), environment=LINUX, requires=REQUIRES, versions={})
    assert report.dimos_version is None and not report.satisfied_for_launch


def test_report_json_round_trip() -> None:
    report = EnvironmentReport(
        python="3.12.1",
        prefix="/venv",
        dimos_version="0.0.14",
        checks=["packages"],
        missing=[RequirementIssue("x>=1", "web", None, "missing")],
        conflicts=[("onnxruntime", "onnxruntime-gpu")],
        native={"dimos_mls_planner": Outcome("missing", "ModuleNotFoundError: x")},
        system={"rclpy": Outcome("satisfied")},
        tools={"ffmpeg": Outcome("missing", "not on PATH")},
        backends={"onnxruntime": Outcome("satisfied", "providers=['CPUExecutionProvider']")},
        blueprints={"unitree-go2": Outcome("satisfied")},
    )
    assert EnvironmentReport.from_json(report.to_json()) == report


def test_ok_needs_every_checked_prerequisite() -> None:
    report = EnvironmentReport(python="3.12.1", prefix="/venv", dimos_version="0.0.14")
    unchecked = Outcome("unchecked", "native executable built separately; not checked here")
    report.native = {"local_planner": unchecked}
    assert report.ok and report.unchecked == [("native", "local_planner", unchecked)]
    missing = Outcome("missing", "ModuleNotFoundError: No module named 'rclpy'")
    report.system = {"rclpy": missing}
    assert not report.ok and report.missing_prerequisites == [("system", "rclpy", missing)]
    report.system = {}
    report.tools = {"deno": Outcome("missing", "not on PATH")}
    assert not report.ok
    report.tools = {}
    report.blueprints = {"unitree-go2": Outcome("missing", "ImportError: x")}
    assert not report.ok and report.satisfied_for_launch


def test_native_executables_are_unchecked_and_host_modules_are_imported() -> None:
    assert check_native(["local_planner"]) == {
        "local_planner": Outcome(
            "unchecked", "native executable built separately; not checked here"
        )
    }
    results = check_system(["json", "no_such_module_xyz"])
    assert results["json"] == Outcome("satisfied")
    assert results["no_such_module_xyz"].status == "missing"
    assert "ModuleNotFoundError" in str(results["no_such_module_xyz"].detail)


def test_format_report_recipes() -> None:
    plan = Plan(extras=frozenset({"web", "agents"}))
    report = check_environment(plan, environment=LINUX, requires=REQUIRES, versions=VERSIONS)
    text = format_report(report, plan, checkout=True)
    assert "uv sync --extra agents --extra web --inexact" in text
    assert "langchain<2,>=1.2.3 (extra agents): not installed" in text
    text = format_report(report, plan, checkout=False)
    assert "pip install 'dimos[agents,web]'" in text
    assert "dimos prepare <blueprint>" in text
    satisfied = check_environment(Plan(), environment=LINUX, requires=REQUIRES, versions=VERSIONS)
    assert "Requirements: satisfied" in format_report(satisfied, Plan(), checkout=True)
    satisfied.system = {"rclpy": Outcome("missing", "ModuleNotFoundError: No module named 'rclpy'")}
    satisfied.native = {"local_planner": Outcome("unchecked", "built separately")}
    satisfied.blueprints = {"unitree-go2": Outcome("satisfied")}
    text = format_report(satisfied, Plan(), checkout=True)
    assert "Host module rclpy: missing (ModuleNotFoundError: No module named 'rclpy')" in text
    assert "Native module local_planner: unchecked (built separately)" in text
    assert "Blueprint unitree-go2: satisfied" in text
