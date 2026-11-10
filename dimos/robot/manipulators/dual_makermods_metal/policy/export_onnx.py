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

"""Export the trained dual Metal actor to a deterministic ONNX artifact.

This is a build-time tool: it is the only code that imports torch. It rebuilds
the actor exactly as ``rsl_rl.models.MLPModel`` defines it for this checkpoint -
an ELU MLP with an ``EmpiricalNormalization`` prefix whose forward pass is
``(x - mean) / (std + eps)`` - loads the trusted checkpoint, exports only the
deterministic mean action, and writes the normalizer plus a hash-pinned
manifest next to the ONNX file. Critic, optimizer and training state never
leave the checkpoint.

Usage:
    python -m dimos.robot.manipulators.dual_makermods_metal.policy.export_onnx \
        --checkpoint /path/checkpoint_038000.pt --out /path/model_dir
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    ACTION_SCALE,
    HISTORY_FRAMES,
    HOME,
    NORMALIZER_EPS,
    POLICY_HZ,
    SCHEMA,
    CheckpointSpec,
    ModelSpec,
    PolicyManifest,
    sha256_file,
)


@dataclass(frozen=True)
class ExportReport:
    """Outcome of one export, suitable for logging into the PR evidence."""

    manifest_path: Path
    onnx_path: Path
    normalizer_path: Path
    checkpoint_sha256: str
    iteration: int
    max_abs_diff: float
    sample_count: int


def _load_reference_actor(checkpoint: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    """Rebuild the checkpoint's actor as a torch module, matching rsl_rl semantics."""
    import torch
    from torch import nn

    raw_state = checkpoint.get("actor_state_dict")
    if not isinstance(raw_state, dict):
        raise ValueError("checkpoint has no actor_state_dict")
    state: dict[str, Any] = raw_state
    expected = {
        "obs_normalizer._mean",
        "obs_normalizer._var",
        "obs_normalizer._std",
        "distribution.std_param",
        "mlp.0.weight",
        "mlp.0.bias",
        "mlp.2.weight",
        "mlp.2.bias",
        "mlp.4.weight",
        "mlp.4.bias",
        "mlp.6.weight",
        "mlp.6.bias",
    }
    missing = expected - set(state)
    if missing:
        raise ValueError(f"checkpoint actor is missing keys: {sorted(missing)}")
    mean = state["obs_normalizer._mean"].to(torch.float32).reshape(1, -1)
    std = state["obs_normalizer._std"].to(torch.float32).reshape(1, -1)
    if mean.shape != (1, HISTORY_FRAMES * 96) or std.shape != mean.shape:
        raise ValueError(f"unexpected normalizer shape {tuple(mean.shape)}")

    class ReferenceActor(nn.Module):
        """``MLPModel`` forward for this checkpoint: normalize, then ELU MLP, mean output."""

        mean: torch.Tensor
        std: torch.Tensor

        def __init__(self) -> None:
            super().__init__()
            self.register_buffer("mean", mean.clone())
            self.register_buffer("std", std.clone())
            layers: list[nn.Module] = []
            widths = [96 * HISTORY_FRAMES, 256, 256, 256, 12]
            for index in range(4):
                linear = nn.Linear(widths[index], widths[index + 1])
                linear.weight.data.copy_(state[f"mlp.{index * 2}.weight"].to(torch.float32))
                linear.bias.data.copy_(state[f"mlp.{index * 2}.bias"].to(torch.float32))
                layers.append(linear)
                if index < 3:
                    layers.append(nn.ELU())
            self.mlp = nn.Sequential(*layers)

        def forward(self, obs: Any) -> Any:
            return self.mlp((obs - self.mean) / (self.std + NORMALIZER_EPS))

    actor = ReferenceActor().eval()
    return actor, state


def _contract_fields(checkpoint: dict[str, Any]) -> dict[str, Any]:
    contract = checkpoint.get("contract")
    if not isinstance(contract, dict):
        raise ValueError("checkpoint has no contract block")
    if contract.get("schema") != "metal-ppo-pd-total-torque-v1":
        raise ValueError(f"unexpected training schema {contract.get('schema')!r}")
    expected = {
        "policy_hz": POLICY_HZ,
        "home": list(HOME),
        "action_scale": ACTION_SCALE,
        "history_frames": HISTORY_FRAMES,
    }
    for key, value in expected.items():
        if contract.get(key) != value:
            raise ValueError(f"checkpoint contract {key}={contract.get(key)!r} != {value!r}")
    recipe = contract.get("recipe_variant") or {}
    variant = str(recipe.get("name", ""))
    if variant != "cmdlp005-ori10-rawex001-v1":
        raise ValueError(f"unexpected recipe variant {variant!r}")
    if "0.05*(desired-issued)" not in str(recipe.get("only_numerical_change", "")):
        raise ValueError("recipe is not the accepted low-pass fork")
    return contract


def export_checkpoint(
    checkpoint_path: str | Path,
    out_dir: str | Path,
    *,
    name: str = "metal_arm_policy",
    opset: int = 17,
    seed: int = 20261109,
) -> ExportReport:
    """Export one checkpoint; raises on any contract or parity violation."""
    import torch

    checkpoint_path = Path(checkpoint_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    contract = _contract_fields(checkpoint)
    actor, state = _load_reference_actor(checkpoint)
    iteration = int(checkpoint["iteration"])
    checkpoint_sha = sha256_file(checkpoint_path)

    mean = state["obs_normalizer._mean"].to(torch.float32).numpy().reshape(-1)
    std = state["obs_normalizer._std"].to(torch.float32).numpy().reshape(-1)
    normalizer_path = out_dir / f"{name}.normalizer.npz"
    np.savez(normalizer_path, mean=mean.astype(np.float32), std=std.astype(np.float32))
    normalizer_sha = sha256_file(normalizer_path)

    onnx_path = out_dir / f"{name}.onnx"
    example = torch.zeros(1, HISTORY_FRAMES * 96, dtype=torch.float32)
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

    manifest = PolicyManifest(
        name=name,
        checkpoint=CheckpointSpec(
            sha256=checkpoint_sha,
            iteration=iteration,
            source_sha256=str(checkpoint.get("source_sha256", "")),
            recipe_variant=str((contract.get("recipe_variant") or {}).get("name", "")),
        ),
        model=ModelSpec(onnx_sha256=onnx_sha, normalizer_sha256=normalizer_sha),
        metadata={
            "onnx_file": onnx_path.name,
            "normalizer_file": normalizer_path.name,
            "exporter": "dimos.robot.manipulators.dual_makermods_metal.policy.export_onnx",
            "exporter_schema": SCHEMA,
            "opset": opset,
            "architecture": "mlp-elu-256-256-256",
            "normalizer_forward": "(x - mean) / (std + eps)",
            "deterministic_output": "distribution mean, no sampling",
            "training_config_sha256": hashlib.sha256(
                json.dumps(checkpoint.get("config"), sort_keys=True).encode()
            ).hexdigest(),
        },
    )
    manifest_path = out_dir / f"{name}.manifest.json"
    manifest.dump(manifest_path)

    # Parity: reference torch actor vs exported ONNX on checkpoint histories and stress inputs.
    # Realistic inputs (recorded histories, unit/observer-scale noise) must match in
    # absolute terms; extreme saturation inputs only within float32 round-off relative
    # to their own magnitude, because reordered matmuls amplify with activation size.
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(seed)
    recorded = (
        checkpoint["env_state"]["history"]
        .to(torch.float32)
        .numpy()
        .reshape(-1, HISTORY_FRAMES * 96)
    )
    picks = rng.choice(recorded.shape[0], size=min(64, recorded.shape[0]), replace=False)
    realistic = [
        recorded[picks],
        rng.normal(0.0, 1.0, size=(32, HISTORY_FRAMES * 96)).astype(np.float32),
        np.zeros((1, HISTORY_FRAMES * 96), dtype=np.float32),
    ]
    saturated = [rng.normal(0.0, 50.0, size=(32, HISTORY_FRAMES * 96)).astype(np.float32)]
    # Boundary cases: saturate one observation slot at a time.
    for slot in (0, 11, 12, 24, 36, 48, 72, HISTORY_FRAMES * 96 - 1):
        sample = recorded[:4].copy()
        sample[:, slot] = 1e4
        saturated.append(sample)
    worst_abs = 0.0
    worst_rel = 0.0
    count = 0
    for index, sample in enumerate(realistic + saturated):
        sample = np.ascontiguousarray(sample, dtype=np.float32)
        with torch.inference_mode():
            reference = actor(torch.from_numpy(sample)).numpy()
        exported = session.run(["action"], {"obs": sample})[0]
        difference = np.abs(reference - exported)
        scale = max(float(np.abs(reference).max()), 1e-6)
        worst_rel = max(worst_rel, float(difference.max()) / scale)
        if index < len(realistic):
            worst_abs = max(worst_abs, float(difference.max()))
        count += sample.shape[0]
    if worst_abs > 1e-4:
        raise AssertionError(f"exported actor diverges on realistic inputs: {worst_abs}")
    if worst_rel > 1e-5:
        raise AssertionError(f"exported actor diverges relatively on saturated inputs: {worst_rel}")
    return ExportReport(
        manifest_path=manifest_path,
        onnx_path=onnx_path,
        normalizer_path=normalizer_path,
        checkpoint_sha256=checkpoint_sha,
        iteration=iteration,
        max_abs_diff=worst_abs,
        sample_count=count,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, help="accepted lineage checkpoint (.pt)")
    parser.add_argument("--out", required=True, help="directory for the ONNX artifact and manifest")
    parser.add_argument("--name", default="metal_arm_policy")
    parser.add_argument("--opset", type=int, default=17)
    options = parser.parse_args()
    report = export_checkpoint(
        options.checkpoint, options.out, name=options.name, opset=options.opset
    )
    print(
        json.dumps(
            {
                "manifest": str(report.manifest_path),
                "onnx": str(report.onnx_path),
                "normalizer": str(report.normalizer_path),
                "checkpoint_sha256": report.checkpoint_sha256,
                "iteration": report.iteration,
                "parity_samples": report.sample_count,
                "max_abs_diff": report.max_abs_diff,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
