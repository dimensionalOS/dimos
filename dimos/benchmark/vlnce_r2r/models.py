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

"""Strict persisted contracts for one VLN-CE R2R Evaluation case."""

from __future__ import annotations

import hashlib
from pathlib import PurePosixPath
from typing import Annotated, Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, model_validator

NonEmpty = Annotated[str, Field(min_length=1)]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
GitSha = Annotated[str, Field(pattern=r"^[0-9a-f]{40}$")]


class VlnceModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


class ExternalAssetFileRef(VlnceModel):
    path: NonEmpty
    sha256: Sha256

    @model_validator(mode="after")
    def path_is_relative(self) -> ExternalAssetFileRef:
        _validate_relative_path(self.path, "asset path")
        return self


class ExternalAssetRef(VlnceModel):
    asset_id: NonEmpty
    url: NonEmpty
    archive_sha256: Sha256
    archive_bytes: int = Field(gt=0)
    cache_subdir: NonEmpty
    required_files: tuple[ExternalAssetFileRef, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def fields_are_safe_and_unique(self) -> ExternalAssetRef:
        parsed = urlsplit(self.url)
        if parsed.scheme != "https" or not parsed.netloc or parsed.username or parsed.password:
            raise ValueError("external asset URL must be unauthenticated HTTPS")
        _validate_relative_path(self.cache_subdir, "asset cache_subdir")
        paths = [item.path for item in self.required_files]
        if len(paths) != len(set(paths)):
            raise ValueError("required file paths must be unique")
        return self


class ExternalOciImageRef(VlnceModel):
    image_name: NonEmpty
    image_digest: Sha256
    build_context: NonEmpty
    build_recipe_sha256: Sha256
    base_image: NonEmpty
    base_image_digest: Sha256

    @model_validator(mode="after")
    def build_context_is_relative(self) -> ExternalOciImageRef:
        _validate_relative_path(self.build_context, "OCI build_context")
        return self


class ExternalBenchmarkPreparationRef(VlnceModel):
    kind: Literal["vlnce_public_assets"] = "vlnce_public_assets"
    revision: NonEmpty
    cache_namespace: NonEmpty
    assets: tuple[ExternalAssetRef, ...] = Field(min_length=1)
    image: ExternalOciImageRef

    @model_validator(mode="after")
    def assets_are_unique(self) -> ExternalBenchmarkPreparationRef:
        asset_ids = [asset.asset_id for asset in self.assets]
        if len(asset_ids) != len(set(asset_ids)):
            raise ValueError("asset IDs must be unique")
        _validate_relative_path(self.cache_namespace, "cache namespace")
        return self


class VlnceEpisodeSource(VlnceModel):
    kind: Literal["external_benchmark_episode"] = "external_benchmark_episode"
    benchmark: Literal["vlnce_r2r"] = "vlnce_r2r"
    upstream_revision: GitSha
    dataset_revision: NonEmpty
    split: NonEmpty
    episode_id: NonEmpty
    episode_sha256: Sha256
    scene_id: NonEmpty
    episode_asset_id: NonEmpty
    episode_path: NonEmpty
    scene_asset_id: NonEmpty
    scene_path: NonEmpty
    navmesh_path: NonEmpty
    condition_label: NonEmpty
    preparation: ExternalBenchmarkPreparationRef

    @model_validator(mode="after")
    def references_are_resolvable(self) -> VlnceEpisodeSource:
        assets = {asset.asset_id for asset in self.preparation.assets}
        if self.episode_asset_id not in assets or self.scene_asset_id not in assets:
            raise ValueError("source references an unknown asset")
        for value, label in (
            (self.episode_path, "episode_path"),
            (self.scene_path, "scene_path"),
            (self.navmesh_path, "navmesh_path"),
        ):
            _validate_relative_path(value, label)
        return self


class BenchmarkInstruction(VlnceModel):
    kind: Literal["benchmark_instruction"] = "benchmark_instruction"
    prompt: NonEmpty
    instruction_sha256: Sha256
    submission_guidance: NonEmpty

    @model_validator(mode="after")
    def instruction_digest_matches(self) -> BenchmarkInstruction:
        if hashlib.sha256(self.prompt.encode()).hexdigest() != self.instruction_sha256:
            raise ValueError("instruction digest does not match prompt")
        return self


class LiveAgentInteraction(VlnceModel):
    kind: Literal["live_agent"] = "live_agent"
    timeout_seconds: float = Field(gt=0, allow_inf_nan=False)


class VlnceTaskManifest(VlnceModel):
    case_id: NonEmpty
    source: VlnceEpisodeSource
    task: BenchmarkInstruction
    interaction: LiveAgentInteraction


class VlnceConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    task_manifest: str = Field(min_length=1)


def _validate_relative_path(value: str, label: str) -> None:
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or ".." in path.parts or path == PurePosixPath("."):
        raise ValueError(f"{label} must be a safe relative path")
