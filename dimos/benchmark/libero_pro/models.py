"""Strict persisted contracts for one LIBERO-PRO task evaluation."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class LiberoModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class SourceIdentity(LiberoModel):
    repository: Literal["Zxy-MLlab/LIBERO-PRO"]
    revision: str = Field(min_length=40, max_length=40)
    dataset_repository: Literal["zhouxueyang/LIBERO-Pro"]
    dataset_revision: str = Field(min_length=40, max_length=40)


class TaskIdentity(LiberoModel):
    suite: str = Field(min_length=1)
    task_order_index: int = Field(ge=0)
    task_index: int = Field(ge=0)
    task_name: str = Field(min_length=1)
    instruction: str = Field(min_length=1)


class AssetReference(LiberoModel):
    repository_path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(gt=0)


class TaskAssets(LiberoModel):
    bddl: AssetReference
    init_states: AssetReference


class EpisodeSelection(LiberoModel):
    debug_init_state_indices: tuple[int, ...]
    scored_init_state_index: int = Field(ge=0)

    @model_validator(mode="after")
    def rows_are_distinct(self) -> EpisodeSelection:
        if len(self.debug_init_state_indices) != 5:
            raise ValueError("exactly five debug init-state indices are required")
        if any(index < 0 for index in self.debug_init_state_indices):
            raise ValueError("debug init-state indices must be non-negative")
        if len(set(self.debug_init_state_indices)) != 5:
            raise ValueError("debug init-state indices must be distinct")
        if self.scored_init_state_index in self.debug_init_state_indices:
            raise ValueError("scored init-state index must not be used for debugging")
        return self


class CameraContract(LiberoModel):
    name: Literal["agentview", "robot0_eye_in_hand"]
    width: Literal[128]
    height: Literal[128]


class ComparisonContract(LiberoModel):
    robot: Literal["Panda"]
    controller: Literal["JOINT_POSITION"]
    cameras: tuple[CameraContract, CameraContract]
    control_frequency_hz: Literal[20]
    settling_ticks: Literal[5]
    horizon_ticks: int = Field(gt=0)
    clock: Literal["continuous_real_time"]
    success: Literal["native_bddl_goal_predicates"]

    @model_validator(mode="after")
    def cameras_are_exact(self) -> ComparisonContract:
        if tuple(camera.name for camera in self.cameras) != (
            "agentview",
            "robot0_eye_in_hand",
        ):
            raise ValueError("the two benchmark camera contracts must be ordered and complete")
        return self


class LiberoTaskManifest(LiberoModel):
    schema_version: Literal["1.0"] = "1.0"
    case_id: str = Field(min_length=1)
    source: SourceIdentity
    task: TaskIdentity
    assets: TaskAssets
    episodes: EpisodeSelection
    contract: ComparisonContract


class LiberoProConfig(LiberoModel):
    task_manifest: str = Field(min_length=1)
