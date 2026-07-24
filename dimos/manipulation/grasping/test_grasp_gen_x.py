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

import inspect
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXError,
    GraspGenXModule,
    _default_factory,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def config(**overrides: object) -> GraspGenXConfig:
    values: dict[str, object] = {
        "checkpoint_path": "/checkpoints",
        "gripper": {
            "extents_open": (0.1, 0.1, 0.1),
            "offset_open": (0, 0, 0),
            "extents_half_open": (0.1, 0.1, 0.1),
            "offset_half_open": (0, 0, 0),
            "fingertip_depth": 0.1,
        },
    }
    values.update(overrides)
    return GraspGenXConfig(**values)  # type: ignore[arg-type]


class _Tensor:
    def __init__(self, value: object) -> None:
        self.value = value

    def detach(self) -> _Tensor:
        return self

    def cpu(self) -> _Tensor:
        return self

    def numpy(self) -> object:
        return self.value


class _Cloud:
    ts = 12.5
    frame_id = "camera"

    def __init__(self) -> None:
        self.pointcloud_tensor = type(
            "TensorCloud", (), {"point": {"positions": _Tensor(np.zeros((1, 3), dtype=np.float32))}}
        )()


def test_messages_round_trip_empty_and_score() -> None:
    value = GraspCandidateArray(Header(3.0, "camera"), [GraspCandidate(Pose(1, 2, 3), 0.25)])
    decoded = GraspCandidateArray.decode(value.encode())
    assert decoded.header.frame_id == "camera"
    assert decoded.header.timestamp == pytest.approx(3.0)
    assert decoded.candidates[0].score == pytest.approx(0.25)
    empty = GraspCandidateArray(Header(3.0, "camera"), [])
    assert GraspCandidateArray.decode(empty.encode()).candidates == []


def test_spec_signature() -> None:
    signature = inspect.signature(GraspGenSpec.propose_grasps)
    assert list(signature.parameters) == ["self", "object_pointcloud"]
    assert signature.parameters["object_pointcloud"].annotation.__name__ == "PointCloud2"
    assert signature.return_annotation is GraspCandidateArray
    assert inspect.signature(GraspGenXModule.propose_grasps).parameters.keys() == {
        "self",
        "object_pointcloud",
    }


def test_adapter_sorts_stably_truncates_and_right_multiplies() -> None:
    calls = 0

    def factory(_: GraspGenXConfig) -> object:
        nonlocal calls
        calls += 1
        return object()

    def inference(_: object, __: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        poses = np.repeat(np.eye(4)[None], 3, axis=0)
        poses[:, 0, 3] = [1, 2, 3]
        return poses, np.array([0.5, 0.5, 0.9])

    cfg = config(
        max_candidates=2,
        grasp_frame_to_tcp=((0, -1, 0, 10), (1, 0, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
    )
    module = GraspGenXModule(cfg, factory=factory, inference=inference)
    module.start()
    module.start()
    result = module.propose_grasps(_Cloud())
    assert calls == 1
    assert [candidate.score for candidate in result] == [0.9, 0.5]
    assert [candidate.pose.position.x for candidate in result] == [13, 11]
    assert result.candidates[0].pose.orientation.z == pytest.approx(2**-0.5)
    assert result.candidates[0].pose.orientation.w == pytest.approx(2**-0.5)
    assert result.candidates[0].pose.position.x == pytest.approx(13)
    assert result.header.timestamp == pytest.approx(12.5)
    module.stop()


def test_empty_backend_and_invalid_inputs() -> None:
    module = GraspGenXModule(
        config(),
        factory=lambda _: object(),
        inference=lambda *_: (np.empty((0, 4, 4)), np.empty(0)),
    )
    module.start()
    result = module.propose_grasps(_Cloud())  # type: ignore[arg-type]
    assert len(result) == 0
    assert result.header.timestamp == pytest.approx(12.5)
    assert result.header.frame_id == "camera"
    module.stop()
    with pytest.raises(ValueError):
        GraspGenXConfig(**{**config().model_dump(), "max_candidates": 0})

    invalid_gripper = config().gripper.model_dump()
    for field, value in {
        "family": "unsupported",
        "extents_open": (1, 2),
        "offset_open": (np.nan, 0, 0),
        "fingertip_depth": 0,
    }.items():
        with pytest.raises(ValueError):
            GraspGenXConfig(
                **{
                    **config().model_dump(),
                    "gripper": {**invalid_gripper, field: value},
                }
            )
    with pytest.raises(ValueError):
        GraspGenXConfig(
            **{
                **config().model_dump(),
                "grasp_frame_to_tcp": ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
            }
        )

    malformed = GraspGenXModule(
        config(), factory=lambda _: object(), inference=lambda *_: (np.ones((1, 4, 4)), np.ones(1))
    )
    malformed.start()
    with pytest.raises(ValueError, match="homogeneous"):
        malformed.propose_grasps(_Cloud())
    malformed.stop()


@pytest.mark.parametrize(
    "points",
    [
        np.array([[1, 2, 3]], dtype=np.int32),
        np.array([[np.nan, 0, 0]], dtype=np.float32),
        np.empty((0, 3), dtype=np.float32),
        np.zeros(3, dtype=np.float32),
    ],
)
def test_invalid_cloud_points(points: np.ndarray) -> None:
    cloud = _Cloud()
    cloud.pointcloud_tensor.point["positions"] = _Tensor(points)
    module = GraspGenXModule(
        config(), factory=lambda _: object(), inference=lambda *_: pytest.fail()
    )
    module.start()
    with pytest.raises(ValueError):
        module.propose_grasps(cloud)
    module.stop()


def test_missing_timestamp_and_not_started() -> None:
    module = GraspGenXModule(config(), factory=lambda _: object())
    with pytest.raises(GraspGenXError, match="not been started"):
        module.propose_grasps(_Cloud())
    module.start()
    cloud = _Cloud()
    cloud.ts = None
    with pytest.raises(ValueError, match="timestamp"):
        module.propose_grasps(cloud)
    module.stop()


@pytest.mark.parametrize(
    "backend",
    [
        (np.ones((2, 4, 4)), np.ones(1)),
        (np.full((1, 4, 4), np.nan), np.ones(1)),
        (np.ones((1, 4, 4)), np.array([np.inf])),
    ],
)
def test_invalid_backend_outputs(backend: tuple[np.ndarray, np.ndarray]) -> None:
    module = GraspGenXModule(config(), factory=lambda _: object(), inference=lambda *_: backend)
    module.start()
    with pytest.raises(ValueError):
        module.propose_grasps(_Cloud())
    module.stop()


def test_backend_tensor_conversion_and_exception_wrapping() -> None:
    poses = _Tensor(np.repeat(np.eye(4, dtype=np.float32)[None], 1, axis=0))
    scores = _Tensor(np.array([0.25], dtype=np.float32))
    seen: list[np.ndarray] = []

    def inference(_: object, points: np.ndarray) -> tuple[_Tensor, _Tensor]:
        seen.append(points)
        return poses, scores

    module = GraspGenXModule(config(), factory=lambda _: object(), inference=inference)
    module.start()
    assert len(module.propose_grasps(_Cloud())) == 1
    assert seen[0].dtype == np.float32
    module.stop()

    failing = GraspGenXModule(
        config(),
        factory=lambda _: object(),
        inference=lambda *_: (_ for _ in ()).throw(RuntimeError("boom")),
    )
    failing.start()
    with pytest.raises(GraspGenXError, match="inference"):
        failing.propose_grasps(_Cloud())
    failing.stop()


def test_factory_exception_wrapping() -> None:
    module = GraspGenXModule(
        config(), factory=lambda _: (_ for _ in ()).throw(RuntimeError("boom"))
    )
    try:
        with pytest.raises(GraspGenXError, match="construction"):
            module.start()
    finally:
        module.stop()


@pytest.mark.parametrize(
    ("family", "gripper_type"),
    [("parallel_2f", 0), ("revolute_2f", 1), ("revolute_3f", 2)],
)
def test_default_factory_uses_direct_upstream_calls(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, family: str, gripper_type: int
) -> None:
    (tmp_path / "gen").mkdir()
    (tmp_path / "dis").mkdir()
    calls: dict[str, object] = {}

    checkpoint_io = ModuleType("graspgenx.utils.checkpoint_io")
    cfg = SimpleNamespace(
        diffusion=SimpleNamespace(gripper_backbone="sweep_volume"),
        discriminator=SimpleNamespace(gripper_backbone="sweep_volume"),
    )
    checkpoint_io.load_model_cfg = lambda *args, **kwargs: calls.update(load=(args, kwargs)) or cfg  # type: ignore[attr-defined]
    x_grippers = ModuleType("graspgenx.x_grippers")
    x_grippers.make_sweep_volume_gripper_info = (  # type: ignore[attr-defined]
        lambda **kwargs: calls.update(gripper_info=kwargs) or "gripper-info"
    )
    server = ModuleType("graspgenx.grasp_server")
    server.SWEEP_VOLUME_ONLY_BACKBONES = {"sweep_volume"}  # type: ignore[attr-defined]

    server.GraspGenXSampler = (  # type: ignore[attr-defined]
        lambda *args, **kwargs: calls.update(constructor=(args, kwargs)) or "sampler"
    )
    monkeypatch.setitem(sys.modules, "graspgenx.utils.checkpoint_io", checkpoint_io)
    monkeypatch.setitem(sys.modules, "graspgenx.x_grippers", x_grippers)
    monkeypatch.setitem(sys.modules, "graspgenx.grasp_server", server)
    for name in ("graspgenx", "graspgenx.utils"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))

    assert (
        _default_factory(
            config(
                checkpoint_path=str(tmp_path),
                gripper={
                    "extents_open": (0.1, 0.2, 0.3),
                    "offset_open": (0.1, 0.2, 0.3),
                    "extents_half_open": (0.1, 0.2, 0.3),
                    "offset_half_open": (0.1, 0.2, 0.3),
                    "fingertip_depth": 0.3,
                    "family": family,
                },
            )
        )
        == "sampler"
    )
    assert calls["load"] == (
        (tmp_path / "gen", tmp_path / "dis"),
        {"gen_pth": None, "dis_pth": None},
    )
    assert calls["gripper_info"] == {
        "extents_open": (0.1, 0.2, 0.3),
        "offset_open": (0.1, 0.2, 0.3),
        "extents_mid": (0.1, 0.2, 0.3),
        "offset_mid": (0.1, 0.2, 0.3),
        "gripper_type": gripper_type,
        "fingertip_depth": 0.3,
    }
    assert calls["constructor"] == ((cfg,), {"gripper_info": "gripper-info"})
    assert not any(name.startswith("graspgenx.serving") for name in sys.modules)


def test_default_factory_rejects_asset_backbone_before_construction(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "gen").mkdir()
    (tmp_path / "dis").mkdir()
    constructed = False
    checkpoint_io = ModuleType("graspgenx.utils.checkpoint_io")
    checkpoint_io.load_model_cfg = lambda *args, **kwargs: SimpleNamespace(  # type: ignore[attr-defined]
        diffusion=SimpleNamespace(gripper_backbone="asset_backbone"),
        discriminator=SimpleNamespace(gripper_backbone="sweep_volume"),
    )
    server = ModuleType("graspgenx.grasp_server")
    server.SWEEP_VOLUME_ONLY_BACKBONES = {"sweep_volume"}  # type: ignore[attr-defined]
    server.GraspGenXSampler = lambda *args, **kwargs: constructed  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "graspgenx.x_grippers", ModuleType("graspgenx.x_grippers"))
    monkeypatch.setitem(sys.modules, "graspgenx.utils.checkpoint_io", checkpoint_io)
    monkeypatch.setitem(sys.modules, "graspgenx.grasp_server", server)
    monkeypatch.setitem(sys.modules, "graspgenx", ModuleType("graspgenx"))
    monkeypatch.setitem(sys.modules, "graspgenx.utils", ModuleType("graspgenx.utils"))

    with pytest.raises(ValueError, match="diffusion.gripper_backbone.*asset-backed"):
        _default_factory(config(checkpoint_path=str(tmp_path)))
    assert not constructed


def test_missing_checkpoint_is_reported() -> None:
    from dimos.manipulation.grasping.grasp_gen_x import _default_factory

    with pytest.raises(FileNotFoundError):
        _default_factory(config(checkpoint_path="/definitely/missing"))


def test_import_is_offline_and_does_not_hydrate_in_child(tmp_path: Path) -> None:
    """A fresh upstream import must only inspect the explicitly local deployment."""
    pytest.importorskip("graspgenx", reason="optional graspgenx extra is not installed")
    checkpoint = tmp_path / "deployment"
    (checkpoint / "gen").mkdir(parents=True)
    (checkpoint / "dis").mkdir()
    fake_git = tmp_path / "git"
    git_marker = tmp_path / "git-invoked"
    fake_git.write_text(f"#!/bin/sh\ntouch {git_marker}\nexit 97\n", encoding="utf-8")
    fake_git.chmod(0o755)
    script = """
import os
from pathlib import Path
import graspgenx
assert os.environ["GRASPGENX_CHECKPOINT_DIR"] == os.environ["CHECKPOINT"]
assert os.environ["GRASPGENX_GRIPPER_CFG_DIR"] == os.environ["CHECKPOINT"]
assert not list(Path(os.environ["CHECKPOINT"]).glob("**/.git"))
print("offline-import-ok")
"""
    env = os.environ.copy()
    env.update(
        {
            "CHECKPOINT": str(checkpoint),
            "GRASPGENX_CHECKPOINT_DIR": str(checkpoint),
            "GRASPGENX_GRIPPER_CFG_DIR": str(checkpoint),
            "PATH": f"{tmp_path}{os.pathsep}{env['PATH']}",
        }
    )
    result = subprocess.run(
        [sys.executable, "-c", script], env=env, capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "offline-import-ok" in result.stdout
    assert not git_marker.exists()
    assert not (checkpoint / ".git").exists()
