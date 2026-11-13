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

"""Export the frozen OpenYAM actor to a deterministic ONNX artifact.

This is a build-time tool: it is the only OpenYAM deployment code that imports
torch. It rebuilds the checkpoint's ``MultiEncoderAE`` for the single
``motion`` task exactly as ``supermimic.models`` defines it - a goal encoder
``19 -> 256 -> 256 -> 32`` with LayerNorm/GELU and a ``tanh`` latent, a decoder
``(32 + 16*20) -> 256 -> 256 -> 6``, and the two ``EmpiricalNormalization``
prefixes whose forward is ``(x - mean) / (std + eps)`` - loads only the actor
state dict, and writes the ONNX graph, the normalizer statistics, a copy of the
frozen reference clip and a hash-pinned manifest beside them.

Usage:
    python -m dimos.robot.manipulators.openyam.policy.export_onnx \
        --actor /path/actor_a00006000.pt --reference-clip /path/clip.npz \
        --out /path/model_dir
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
from typing import Any

import numpy as np

from dimos.robot.manipulators.openyam.policy.contract import (
    ACTION_SCALE,
    ANCHOR_TOLERANCE,
    GOAL_DIM,
    HISTORY_FRAMES,
    NORMALIZER_EPS,
    SCHEMA,
    CheckpointSpec,
    ModelSpec,
    PolicyContractError,
    PolicyManifest,
    ReferenceSpec,
    sha256_file,
    write_manifest,
)

#: Parameter keys of the trained actor, by module, for the single ``motion`` task.
ENCODER_KEYS = (
    "0.weight",
    "0.bias",
    "1.weight",
    "1.bias",
    "3.weight",
    "3.bias",
    "4.weight",
    "4.bias",
    "6.weight",
    "6.bias",
)
DECODER_KEYS = ENCODER_KEYS


@dataclass(frozen=True)
class ExportReport:
    """Outcome of one export, suitable for the PR evidence."""

    manifest_path: Path
    onnx_path: Path
    normalizer_path: Path
    reference_path: Path | None
    checkpoint_sha256: str
    actor_updates: int
    max_abs_diff: float
    sample_count: int
    onnx_sha256: str


def build_reference_actor(state: dict[str, Any]) -> Any:
    """Rebuild the checkpoint actor as a torch module with the trained forward."""
    import torch
    from torch import nn

    frame_dim = sum(
        width for name, width in (("q", 6), ("dq", 6), ("issued", 6), ("valid", 1), ("time_s", 1))
    )
    assert frame_dim == 20

    def mlp(widths: tuple[int, ...]) -> nn.Sequential:
        layers: list[nn.Module] = []
        for index in range(len(widths) - 1):
            layers.append(nn.Linear(widths[index], widths[index + 1]))
            if index < len(widths) - 2:
                layers.append(nn.LayerNorm(widths[index + 1]))
                layers.append(nn.GELU())
        return nn.Sequential(*layers)

    class ReferenceActor(nn.Module):
        """``MultiEncoderAE`` forward for one ``motion`` task, mean action only."""

        def __init__(self) -> None:
            super().__init__()
            self.register_buffer("obs_mean", state["obs_normalizer._mean"].float().clone())
            self.register_buffer("obs_std", state["obs_normalizer._std"].float().clone())
            self.register_buffer(
                "goal_mean", state["goal_normalizers.motion._mean"].float().clone()
            )
            self.register_buffer("goal_std", state["goal_normalizers.motion._std"].float().clone())
            self.encoder = mlp((GOAL_DIM, 256, 256, 32))
            self.decoder = mlp((32 + HISTORY_FRAMES * frame_dim, 256, 256, 6))
            encoder_state = {
                key.removeprefix("encoders.motion."): value
                for key, value in state.items()
                if key.startswith("encoders.motion.")
            }
            decoder_state = {
                key.removeprefix("decoder."): value
                for key, value in state.items()
                if key.startswith("decoder.")
            }
            self.encoder.load_state_dict(encoder_state, strict=True)
            self.decoder.load_state_dict(decoder_state, strict=True)

        def forward(self, obs: Any) -> Any:
            history = obs[:, : HISTORY_FRAMES * frame_dim].reshape(-1, HISTORY_FRAMES, frame_dim)
            goal = obs[:, HISTORY_FRAMES * frame_dim :]
            history_n = (history - self.obs_mean) / (self.obs_std + NORMALIZER_EPS)
            goal_n = (goal - self.goal_mean) / (self.goal_std + NORMALIZER_EPS)
            latent = torch.tanh(self.encoder(goal_n))
            return self.decoder(torch.cat([latent, history_n.flatten(start_dim=1)], dim=-1))

    actor = ReferenceActor().eval()
    unused = set(state) - {
        "obs_normalizer._mean",
        "obs_normalizer._std",
        "obs_normalizer._var",
        "obs_normalizer.count",
        "goal_normalizers.motion._mean",
        "goal_normalizers.motion._std",
        "goal_normalizers.motion._var",
        "goal_normalizers.motion.count",
        *(f"encoders.motion.{key}" for key in ENCODER_KEYS),
        *(f"decoder.{key}" for key in DECODER_KEYS),
    }
    if unused:
        raise ValueError(f"actor state has unexpected keys: {sorted(unused)}")
    for key in ("_var", "count"):
        for prefix in ("obs_normalizer.", "goal_normalizers.motion."):
            if f"{prefix}{key}" not in state:
                raise ValueError(f"actor state is missing {prefix}{key}")
    return actor


def _parity(
    actor: Any, session: Any, inputs: list[np.ndarray], *, tolerance: float = 1e-4
) -> tuple[float, int]:
    """Compare the reference torch actor with the exported graph."""
    import torch

    worst = 0.0
    count = 0
    for sample in inputs:
        batch = np.ascontiguousarray(sample, dtype=np.float32)
        with torch.inference_mode():
            reference = actor(torch.from_numpy(batch)).numpy()
        exported = session.run(["action"], {"obs": batch})[0]
        worst = max(worst, float(np.abs(reference - exported).max()))
        count += int(batch.shape[0])
    if worst > tolerance:
        raise AssertionError(f"exported actor diverges from the checkpoint: {worst}")
    return worst, count


def export_actor(
    actor_path: str | Path,
    out_dir: str | Path,
    *,
    name: str = "openyam_policy",
    opset: int = 17,
    reference_clip: str | Path | None = None,
    parity_history: str | Path | None = None,
    source_sha256: str = "",
    plant_model: str | Path | None = None,
    seed: int = 20261111,
) -> ExportReport:
    """Export one frozen actor; raises on any contract or parity violation."""
    import onnxruntime as ort
    import torch

    actor_path = Path(actor_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    payload = torch.load(actor_path, map_location="cpu", weights_only=False)
    state = payload["actor"]
    meta = payload["meta"]
    actor = build_reference_actor(state)
    actor_updates = int(meta["count"]["actor_updates"])

    obs_mean = state["obs_normalizer._mean"].float().numpy().reshape(-1)
    obs_std = state["obs_normalizer._std"].float().numpy().reshape(-1)
    goal_mean = state["goal_normalizers.motion._mean"].float().numpy().reshape(-1)
    goal_std = state["goal_normalizers.motion._std"].float().numpy().reshape(-1)
    normalizer_path = out_dir / f"{name}.normalizer.npz"
    np.savez(
        normalizer_path,
        obs_mean=obs_mean.astype(np.float32),
        obs_std=obs_std.astype(np.float32),
        goal_mean=goal_mean.astype(np.float32),
        goal_std=goal_std.astype(np.float32),
    )
    normalizer_sha = sha256_file(normalizer_path)

    obs_dim = HISTORY_FRAMES * 20 + GOAL_DIM
    onnx_path = out_dir / f"{name}.onnx"
    example = torch.zeros(1, obs_dim, dtype=torch.float32)
    torch.onnx.export(
        actor,
        (example,),
        onnx_path,
        input_names=["obs"],
        output_names=["action"],
        opset_version=opset,
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        do_constant_folding=True,
        dynamo=False,
    )
    onnx_sha = sha256_file(onnx_path)
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])

    rng = np.random.default_rng(seed)
    inputs = [
        np.zeros((1, obs_dim), dtype=np.float32),
        rng.normal(0.0, 1.0, size=(32, obs_dim)).astype(np.float32),
    ]
    if parity_history is not None:
        with np.load(parity_history, allow_pickle=False) as data:
            history = np.asarray(data["history"], dtype=np.float32).reshape(-1, HISTORY_FRAMES * 20)
        goal = np.zeros((history.shape[0], GOAL_DIM), dtype=np.float32)
        goal[:, 3] = 1.0  # unit quaternion w
        inputs.append(np.concatenate([history, goal], axis=1))
    worst, count = _parity(actor, session, inputs)

    reference_path: Path | None = None
    reference_spec: ReferenceSpec | None = None
    if reference_clip is not None:
        source = Path(reference_clip)
        reference_path = out_dir / "reference_clip.npz"
        shutil.copyfile(source, reference_path)
        reference_spec = ReferenceSpec(clip_sha256=sha256_file(reference_path))

    plant_metadata: dict[str, Any] = {}
    if plant_model is not None:
        plant = Path(plant_model).resolve()
        if not plant.is_file():
            raise PolicyContractError(f"plant model not found: {plant}")
        plant_metadata = {
            "plant_model": str(plant),
            "plant_sha256": sha256_file(plant),
        }

    manifest = PolicyManifest(
        schema=SCHEMA,
        name=name,
        checkpoint=CheckpointSpec(
            checkpoint_sha256=str(meta["checkpoint_sha256"]),
            actor_updates=actor_updates,
            identity=str(meta["identity"]),
            extraction_sha256=sha256_file(actor_path),
            source_sha256=source_sha256,
        ),
        model=ModelSpec(onnx_sha256=onnx_sha, normalizer_sha256=normalizer_sha),
        reference=reference_spec,
        metadata={
            "onnx_file": onnx_path.name,
            "normalizer_file": normalizer_path.name,
            "exporter": "dimos.robot.manipulators.openyam.policy.export_onnx",
            "exporter_schema": SCHEMA,
            "opset": opset,
            "architecture": "goal19-256-256-32-tanh|decoder352-256-256-6",
            "normalizer_forward": "(x - mean) / (std + eps)",
            "deterministic_output": "distribution mean, no sampling",
            "action_scale": ACTION_SCALE,
            "anchor_tolerance": ANCHOR_TOLERANCE,
            "torch_version": torch.__version__,
            "onnxruntime_version": ort.__version__,
            "parity_max_abs": worst,
            "parity_samples": count,
            "checkpoint_extraction": str(meta.get("extraction", "")),
            **plant_metadata,
        },
    )
    manifest_path = out_dir / f"{name}.manifest.json"
    write_manifest(manifest, manifest_path)
    return ExportReport(
        manifest_path=manifest_path,
        onnx_path=onnx_path,
        normalizer_path=normalizer_path,
        reference_path=reference_path,
        checkpoint_sha256=str(meta["checkpoint_sha256"]),
        actor_updates=actor_updates,
        max_abs_diff=worst,
        sample_count=count,
        onnx_sha256=onnx_sha,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--actor", required=True, help="actor-only extraction (.pt)")
    parser.add_argument("--out", required=True, help="output artifact directory")
    parser.add_argument("--reference-clip", help="frozen reference motion (.npz)")
    parser.add_argument("--parity-history", help="recorded history npz for parity")
    parser.add_argument("--source-sha256", default="", help="hash of the training entry point")
    parser.add_argument(
        "--plant",
        help="trained MuJoCo plant model; pinned so the goal frame and gains travel with the artifact",
    )
    parser.add_argument("--name", default="openyam_policy")
    parser.add_argument("--opset", type=int, default=17)
    options = parser.parse_args()
    report = export_actor(
        options.actor,
        options.out,
        name=options.name,
        opset=options.opset,
        reference_clip=options.reference_clip,
        parity_history=options.parity_history,
        source_sha256=options.source_sha256,
        plant_model=options.plant,
    )
    print(
        json.dumps(
            {
                "manifest": str(report.manifest_path),
                "onnx": str(report.onnx_path),
                "onnx_sha256": report.onnx_sha256,
                "normalizer": str(report.normalizer_path),
                "reference": str(report.reference_path),
                "checkpoint_sha256": report.checkpoint_sha256,
                "actor_updates": report.actor_updates,
                "parity_samples": report.sample_count,
                "max_abs_diff": report.max_abs_diff,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
