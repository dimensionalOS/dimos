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

"""LeRobot `PolicyBackend` adapter.

Bridges `PolicyObservation` ↔ LeRobot's framework-specific inference flow:

    observation → contract.from_messages → preprocess → policy.select_action
    → postprocess → action vector → contract.to_command → JointPositionCommand

`lerobot` is imported lazily inside `initialize()` so the policy package
stays importable on machines where the `datasets` extra is not installed.
The contract layer (already used by the rrd → LeRobot converter) handles
the per-robot observation/action shape, so this file is robot-agnostic.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.manipulation.policy.command import JointPositionCommand, NoOpCommand, PolicyCommand
from dimos.manipulation.policy.contract import RobotContract
from dimos.manipulation.policy.contracts.registry import get_contract
from dimos.manipulation.policy.observation import PolicyObservation

if TYPE_CHECKING:
    import numpy as np


class LeRobotBackend:
    """Run a LeRobot policy against `PolicyObservation` inputs.

    Args:
        contract: Either a `RobotContract` instance or the registered
            contract name (e.g., ``"piper"``). The contract performs the
            observation → frame and action → `JointState` translations and
            knows the action vector layout for the target robot.
        policy_path: Local directory or HuggingFace Hub repo id to load the
            LeRobot policy from. Required.
        policy_kwargs: Additional kwargs forwarded to
            ``Policy.from_pretrained(...)``.
        device: Torch device (``"cuda"``, ``"cpu"``). When ``None``, defers
            to LeRobot's default selection.

    Configuration errors (missing model id, unknown contract, mismatched
    action dimensions) surface from `initialize()` / `select_action()` —
    the node validates the resulting `PolicyCommand` before publishing.
    """

    def __init__(
        self,
        *,
        contract: RobotContract | str,
        policy_path: str | None = None,
        policy_kwargs: dict[str, Any] | None = None,
        device: str | None = None,
    ) -> None:
        if not policy_path:
            raise ValueError(
                "LeRobotBackend: 'policy_path' is required (HF repo id or local directory)"
            )

        self._contract: RobotContract = (
            contract if isinstance(contract, RobotContract) else get_contract(contract)
        )
        self._policy_path = policy_path
        self._policy_kwargs: dict[str, Any] = dict(policy_kwargs or {})
        self._device = device

        self._policy: Any = None  # set in initialize()
        self._torch: Any = None  # cached torch module

    def initialize(self) -> None:
        """Lazily import LeRobot, load the policy, move it to the chosen
        device, and put it in eval mode."""
        import torch  # type: ignore[import-not-found, import-untyped]

        Policy = self._import_policy_class()
        policy = Policy.from_pretrained(self._policy_path, **self._policy_kwargs)

        if self._device is not None:
            policy = policy.to(self._device)
        policy.eval()

        self._policy = policy
        self._torch = torch
        # Discard any state the constructor may have buffered.
        self._reset_policy_state()

    def select_action(self, observation: PolicyObservation) -> PolicyCommand:
        if self._policy is None:
            raise RuntimeError("LeRobotBackend.select_action called before initialize()")
        if observation.joint_state is None:
            return NoOpCommand(reason="no joint_state observed yet")

        try:
            frame = self._contract.from_messages(
                images=observation.images,
                state=observation.joint_state,
                task=observation.task,
            )
        except KeyError as exc:
            return NoOpCommand(reason=f"missing observation feature: {exc}")

        batch = self._frame_to_batch(frame)

        torch = self._torch
        with torch.inference_mode():
            action = self._policy.select_action(batch)

        action_np = self._action_to_numpy(action)
        joint_state = self._contract.to_command(action_np)

        positions = list(joint_state.position)
        names = list(joint_state.name)
        if len(names) != len(positions):
            raise ValueError(
                "LeRobotBackend: contract.to_command produced mismatched names/positions "
                f"(names={len(names)}, positions={len(positions)})"
            )
        return JointPositionCommand(
            joint_names=tuple(names),
            positions=tuple(float(p) for p in positions),
        )

    def reset(self) -> None:
        """Drop any buffered action chunk / recurrent state.

        LeRobot's diffusion- and ACT-style policies expose `reset()` to
        flush their queued action chunks; calling it here guarantees the
        next `select_action()` recomputes from the next observation.
        """
        self._reset_policy_state()

    def close(self) -> None:
        # Drop references so cuda allocations can be reclaimed.
        self._policy = None
        self._torch = None

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _import_policy_class() -> Any:
        try:
            from lerobot.policies.factory import (  # type: ignore[import-not-found, import-untyped]
                get_policy_class,
            )

            return _PolicyAutoLoader(get_policy_class)
        except ImportError:
            pass
        try:
            from lerobot.common.policies.factory import (  # type: ignore[import-not-found, import-untyped]
                get_policy_class,
            )

            return _PolicyAutoLoader(get_policy_class)
        except ImportError as exc:
            raise ImportError(
                "LeRobotBackend requires `lerobot`. Install the datasets extra: "
                "pip install dimos[datasets]"
            ) from exc

    def _reset_policy_state(self) -> None:
        if self._policy is not None and hasattr(self._policy, "reset"):
            self._policy.reset()

    def _frame_to_batch(self, frame: dict[str, Any]) -> dict[str, Any]:
        """Convert a per-tick frame dict into LeRobot batch tensors.

        Image entries gain a leading batch dim and are normalized to
        ``[0, 1]`` float32 in CHW layout. State and other vector entries
        gain a leading batch dim and are kept on the configured device.
        """
        torch = self._torch
        batch: dict[str, Any] = {}
        for key, value in frame.items():
            if key == "task":
                batch[key] = value
                continue
            tensor = self._to_tensor(value)
            if key.startswith("observation.images.") and tensor.ndim == 3:
                # HWC uint8 → CHW float32 in [0, 1]
                tensor = tensor.permute(2, 0, 1).contiguous().to(torch.float32) / 255.0
            tensor = tensor.unsqueeze(0)
            if self._device is not None:
                tensor = tensor.to(self._device)
            batch[key] = tensor
        return batch

    def _to_tensor(self, value: Any) -> Any:
        torch = self._torch
        if torch is None:
            raise RuntimeError("LeRobotBackend: torch not initialized")
        if hasattr(value, "shape") and hasattr(value, "dtype") and not isinstance(value, str):
            # numpy ndarray
            return torch.from_numpy(value)
        return torch.as_tensor(value)

    def _action_to_numpy(self, action: Any) -> np.ndarray:
        import numpy as np

        if hasattr(action, "detach"):  # torch.Tensor
            arr = action.detach().to("cpu").numpy()
        else:
            arr = np.asarray(action)

        # Strip a leading batch dim if present.
        if arr.ndim == 2 and arr.shape[0] == 1:
            arr = arr[0]
        elif arr.ndim > 1:
            raise ValueError(
                "LeRobotBackend: expected action vector or single-batch action, "
                f"got shape {tuple(arr.shape)}"
            )
        return arr.astype(np.float32, copy=False)  # type: ignore[no-any-return]


class _PolicyAutoLoader:
    """Resolve a LeRobot policy class from a saved checkpoint.

    The loader prefers the concrete-class path because lerobot's
    `PreTrainedPolicy.from_pretrained` returns the abstract base in some
    versions (notably 0.5.x), which then fails at instantiation time
    rather than at import time:

    1. Read `path/config.json` for the policy type (top-level `type` in
       lerobot 0.5+, or legacy `policy.type` in older schemas).
    2. If found, dispatch via `get_policy_class(type).from_pretrained(...)`.
    3. Otherwise fall back to `PreTrainedPolicy.from_pretrained(...)`,
       which works on versions where it correctly dispatches to the
       concrete class.
    """

    def __init__(self, get_policy_class: Any) -> None:
        self._get_policy_class = get_policy_class

    def from_pretrained(self, path: str, **kwargs: Any) -> Any:
        policy_type = self._read_policy_type(path)
        if policy_type is not None:
            policy_cls = self._get_policy_class(policy_type)
            return policy_cls.from_pretrained(path, **kwargs)

        # No type in config — last-resort PreTrainedPolicy path.
        try:
            from lerobot.policies.pretrained import (  # type: ignore[import-not-found, import-untyped]
                PreTrainedPolicy,
            )

            return PreTrainedPolicy.from_pretrained(path, **kwargs)
        except ImportError:
            pass
        from lerobot.common.policies.pretrained import (  # type: ignore[import-not-found, import-untyped]
            PreTrainedPolicy,
        )

        return PreTrainedPolicy.from_pretrained(path, **kwargs)

    @staticmethod
    def _read_policy_type(path: str) -> str | None:
        """Extract the policy type from a checkpoint's config.json.

        Handles both schemas: lerobot 0.5+ has `type` at the top level;
        older versions nested it under `policy.type`.
        """
        import json
        from pathlib import Path

        cfg_path = Path(path) / "config.json"
        if not cfg_path.is_file():
            return None
        try:
            data = json.loads(cfg_path.read_text())
        except (OSError, json.JSONDecodeError):
            return None
        if not isinstance(data, dict):
            return None
        # New schema: top-level "type".
        t = data.get("type")
        if isinstance(t, str):
            return t
        # Legacy schema: nested under "policy".
        policy = data.get("policy")
        if isinstance(policy, dict):
            t = policy.get("type")
            if isinstance(t, str):
                return t
        return data.get("type") if isinstance(data, dict) else None


__all__ = ["LeRobotBackend"]
