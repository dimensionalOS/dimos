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

from __future__ import annotations

from email.message import Message
from pathlib import Path
import shlex
import sys
from unittest.mock import Mock

from packaging import markers
import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main, normalize_argv
from dimos.core import run_registry


@pytest.mark.parametrize(
    ("args", "missing", "extra"),
    [
        (["--replay", "run", "unitree-go2"], "gtsam", "mapping"),
        (
            [
                "--zenoh-mode",
                "peer",
                "--zenoh-connect",
                "tcp/127.0.0.1:1",
                "--zenoh-connect-timeout",
                "0",
                "--no-zenoh-scouting",
                "--replay",
                "run",
                "go2-nav-viewer-mesh",
            ],
            "warp",
            "cuda",
        ),
        (["run", "vlm-agent"], "langchain", "agents"),
        (["run", "spatial-memory"], "chromadb", "perception"),
        (["run", "coordinator-flowbase"], "portal", "misc"),
        (["run", "keyboard-teleop-module"], "pygame", "control"),
        (["run", "g1-whole-body-connection"], "unitree_sdk2py", "unitree-dds"),
        (["run", "websocket-vis-module"], "socketio", "web"),
    ],
)
def test_run_reports_missing_extra_before_allocating_workers(
    monkeypatch: pytest.MonkeyPatch, args: list[str], missing: str, extra: str
) -> None:
    monkeypatch.setitem(sys.modules, missing, None)
    allocation = Mock(side_effect=SystemExit(99))
    monkeypatch.setattr(run_registry, "cleanup_stale", allocation)

    result = CliRunner().invoke(main, normalize_argv(["dimos", "--viewer", "none", *args])[1:])

    assert result.exit_code == 1, result.output
    assert f"dimos[{extra}]" in result.output
    allocation.assert_not_called()


def test_explain_missing_extra_reads_provider_and_markers_from_metadata(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from dimos.utils import optional_dependencies

    (tmp_path / "vendor_sdk.py").write_text("raise RuntimeError('imported')\n")
    monkeypatch.syspath_prepend(tmp_path)
    optional_dependencies.check_imports("vendor_sdk")
    assert "vendor_sdk" not in sys.modules

    env = markers.default_environment()
    env["sys_platform"] = "linux"
    env["platform_machine"] = "x86_64"
    env["python_version"] = "3.10"
    env["python_full_version"] = "3.10.12"
    monkeypatch.setattr(markers, "default_environment", lambda: env.copy())
    monkeypatch.setattr(optional_dependencies, "packages_distributions", dict)
    monkeypatch.setitem(sys.modules, "gtsam", None)
    (tmp_path / ".git").touch()
    monkeypatch.setattr(optional_dependencies, "DIMOS_PROJECT_ROOT", tmp_path)

    def fake_dist(req: str) -> Mock:
        meta = Message()
        meta["Provides-Extra"] = "mapping"
        return Mock(metadata=meta, requires=[req])

    monkeypatch.setattr(
        optional_dependencies,
        "distribution",
        lambda _: fake_dist(
            'gtsam-extended>=4.3a1; python_full_version >= "3.11" and extra == "mapping"'
        ),
    )
    with pytest.raises(
        optional_dependencies.MissingExtraError, match="no compatible build"
    ) as incompatible:
        optional_dependencies.check_imports("gtsam")
    assert "dimos[mapping]" in str(incompatible.value)

    monkeypatch.setattr(
        optional_dependencies,
        "distribution",
        lambda _: fake_dist('gtsam>=4.3a1; extra == "mapping"'),
    )
    with pytest.raises(
        optional_dependencies.MissingExtraError, match=r"Install 'dimos\[mapping\]'"
    ) as compatible:
        optional_dependencies.check_imports("gtsam")
    command = str(compatible.value).split("`")[1]
    args = shlex.split(command)
    assert args[args.index("--project") + 1] == str(optional_dependencies.DIMOS_PROJECT_ROOT)


def test_mesh_requires_torch_when_warp_is_present(monkeypatch: pytest.MonkeyPatch) -> None:
    from types import ModuleType

    from dimos.mapping.experimental.mesh import MeshModule

    monkeypatch.setitem(sys.modules, "warp", ModuleType("warp"))
    monkeypatch.setitem(sys.modules, "torch", None)
    with pytest.raises(ImportError, match="torch"):
        MeshModule.check_dependencies()


def test_keyboard_reports_unusable_pygame(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    import runpy

    from dimos.teleop.keyboard import keyboard_teleop_module

    (tmp_path / "pygame.py").write_text("raise ImportError('missing SDL library')\n")
    monkeypatch.syspath_prepend(tmp_path)
    monkeypatch.delitem(sys.modules, "pygame", raising=False)
    module = runpy.run_path(keyboard_teleop_module.__file__)
    with pytest.raises(ImportError, match="missing SDL library"):
        module["KeyboardTeleopModule"].check_dependencies()


def test_spatial_memory_uses_injected_or_in_memory_database(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos.perception.experimental.spatial_perception import SpatialMemory
    from dimos.perception.experimental.spatial_vector_db import SpatialVectorDB

    monkeypatch.setitem(sys.modules, "chromadb", None)

    with pytest.raises(ImportError, match="chromadb"):
        SpatialMemory.check_dependencies(db_path=None)
    client = Mock()
    SpatialMemory.check_dependencies(chroma_client=client)
    db = SpatialVectorDB(chroma_client=client, visual_memory=object())
    assert db.client is client


@pytest.mark.parametrize(
    ("options", "missing"),
    [({}, "langchain_openai"), ({"model": "anthropic:local-example"}, "langchain_anthropic")],
)
def test_vlm_checks_selected_integration(monkeypatch: pytest.MonkeyPatch, options, missing) -> None:
    from dimos.agents.vlm_agent import VLMAgent

    monkeypatch.setitem(sys.modules, missing, None)
    with pytest.raises(ImportError, match=missing.replace("_", "-")):
        VLMAgent.check_dependencies(**options)
