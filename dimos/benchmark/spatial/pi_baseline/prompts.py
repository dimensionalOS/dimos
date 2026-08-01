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

# Copyright 2026 Dimensional Inc.
"""Paired, offline-only baseline prompts and parity manifests."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from typing import Literal

from pydantic import Field

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.utilities import canonical_json

_SHARED = """You are evaluating the supplied case only. Do not use online information or services to solve the task; package installation is allowed. Base Dimos is preinstalled. Additional packages may be installed with `uv pip install` or `python -m pip install`; installs are ephemeral to the current attempt. Do not create environments under /work. The staged case inputs are available under /input; /input is read-only and /work is writable. Write generated files and other artifacts only under /work. Use only these three tools: sandbox_exec, read_generated_image, and submit_answer. You must call submit_answer exactly once before finishing, with the typed answer.
"""
_VISUALIZATION_FORBIDDEN = "Visualization is forbidden. Do not call `read_generated_image`."
_VISUALIZATION_ENCOURAGED = "Visualization is required for acceptance. You have at most two sandbox_exec calls before the first image read. Use the first call to inspect the case. Your second sandbox_exec call must create the final useful PNG at `/work/map.png`; do not spend it only inspecting inputs. Use the exact import `from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2`, then decode with `PointCloud2.lcm_decode(open('/input/maps/map.lcm','rb').read()).points_f32()` and render the x/y points as a useful top-down map no larger than 1024x1024. Do not render raw bytes or metadata. Then call `read_generated_image` with the relative path `map.png`. The image must be an existing regular non-symlink PNG that can be decoded successfully and must satisfy the applicable configured byte, width, height, and pixel limits. If that read fails, use the permitted repair sandbox call to fix `/work/map.png`, then read `map.png` once more. After a successful read, stop exploration: do not run more analysis commands or generate more images. Use the available evidence and call `submit_answer` exactly once with your best answer, even if uncertain. Do not consume remaining turn or tool budget seeking confidence."

PromptMode = Literal["visualization_forbidden", "visualization_encouraged"]


@dataclass(frozen=True)
class PromptPair:
    visualization_forbidden: str
    visualization_encouraged: str


def build_prompt_pair() -> PromptPair:
    """Build prompts whose only difference is the visualization instruction."""
    return PromptPair(_SHARED + _VISUALIZATION_FORBIDDEN, _SHARED + _VISUALIZATION_ENCOURAGED)


class ParityManifest(SpatialModel):
    record_type: Literal["pi-parity-manifest"] = "pi-parity-manifest"
    schema_version: Literal["1.0"] = "1.0"
    model_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    tool_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    runtime_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    dependency_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    mode: PromptMode
    prompt_digest: str = Field(pattern=r"^[0-9a-f]{64}$")


def make_parity_manifest(
    *,
    model: object,
    tools: object,
    runtime: object,
    dependencies: object,
    mode: PromptMode,
    prompt: str,
) -> ParityManifest:
    def digest(value: object) -> str:
        return hashlib.sha256(canonical_json(value)).hexdigest()  # type: ignore[arg-type]

    return ParityManifest(
        model_digest=digest(model),
        tool_digest=digest(tools),
        runtime_digest=digest(runtime),
        dependency_digest=digest(dependencies),
        mode=mode,
        prompt_digest=digest(prompt),
    )


def validate_parity(left: ParityManifest, right: ParityManifest) -> None:
    """Require identical model/tool/runtime/dependency inputs for both modes."""
    for field in ("model_digest", "tool_digest", "runtime_digest", "dependency_digest"):
        if getattr(left, field) != getattr(right, field):
            raise ValueError(f"paired parity mismatch in {field}")
    if left.mode == right.mode:
        raise ValueError("paired manifests must have distinct modes")
