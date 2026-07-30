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

"""Behavior of the focused ``dimos run`` field-flag matcher."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field
import pytest

from dimos.cli.config_flags import (
    FlagResolutionError,
    expand_field_flags,
    iter_config_leaves,
    split_run_tokens,
)
from dimos.cli.dimos import load_config_args
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig


class MapperConfig(ModuleConfig):
    map_file: str | None = None
    publish_loaded_map: bool = True
    voxel_size: float = 0.05


class Mapper(Module):
    config: MapperConfig


class PlannerConfig(ModuleConfig):
    map_file: str | None = None
    lookahead: float = 1.0


class Planner(Module):
    config: PlannerConfig


class StageConfig(BaseModel):
    threshold: float = 0.5


class SingleRefinerConfig(ModuleConfig):
    stage: StageConfig = Field(default_factory=StageConfig)


class SingleRefiner(Module):
    config: SingleRefinerConfig


class DisabledBackend(BaseModel):
    backend: Literal["disabled"] = "disabled"


class EnabledBackend(BaseModel):
    backend: Literal["enabled"] = "enabled"
    level: int = 1


class SwitchConfig(ModuleConfig):
    nested: DisabledBackend | EnabledBackend = Field(default_factory=EnabledBackend)


class Switch(Module):
    config: SwitchConfig


def expand(blueprint: Blueprint, tokens: list[str]) -> list[str]:
    leaves = list(iter_config_leaves(blueprint.config(), blueprint))
    return expand_field_flags(tokens, leaves)


def resolve(blueprint: Blueprint, tokens: list[str], path: Path, *extra: str) -> dict:
    return load_config_args(blueprint.config(), [*expand(blueprint, tokens), *extra], path)


def test_unique_leaf_flag_lands_in_its_own_module(tmp_path: Path) -> None:
    blueprint = autoconnect(Mapper.blueprint(), Planner.blueprint())

    assert expand(blueprint, ["--lookahead", "2.5"]) == ["planner.lookahead=2.5"]

    kwargs = resolve(blueprint, ["--lookahead", "2.5"], tmp_path / "config.json")
    assert kwargs == {"planner": {"lookahead": "2.5"}}


def test_unique_nested_leaf_flag_uses_its_full_config_path(tmp_path: Path) -> None:
    blueprint = SingleRefiner.blueprint()

    assert expand(blueprint, ["--threshold", "0.9"]) == ["singlerefiner.stage.threshold=0.9"]

    kwargs = resolve(blueprint, ["--threshold", "0.9"], tmp_path / "config.json")
    assert kwargs == {"singlerefiner": {"stage": {"threshold": "0.9"}}}


def test_ambiguous_flag_lists_every_candidate_address_flag() -> None:
    blueprint = autoconnect(Mapper.blueprint(), Planner.blueprint())

    with pytest.raises(FlagResolutionError) as error:
        expand(blueprint, ["--map-file", "office"])

    assert str(error.value).splitlines() == [
        "--map-file matches multiple config fields; use a full address flag:",
        "  --mapper.map-file",
        "  --planner.map-file",
    ]


def test_address_flag_reaches_an_ambiguous_field() -> None:
    blueprint = autoconnect(Mapper.blueprint(), Planner.blueprint())

    assert expand(blueprint, ["--planner.map-file", "office"]) == ["planner.map_file=office"]


def test_union_variant_fields_never_match_a_flag() -> None:
    with pytest.raises(FlagResolutionError, match="unknown configuration field flag: --level"):
        expand(Switch.blueprint(), ["--level", "2"])


def test_boolean_field_uses_the_standard_field_flag_syntax(tmp_path: Path) -> None:
    blueprint = Mapper.blueprint()

    kwargs = resolve(blueprint, ["--publish-loaded-map", "false"], tmp_path / "config.json")
    assert MapperConfig(**kwargs["mapper"]).publish_loaded_map is False


def test_missing_value_errors() -> None:
    with pytest.raises(FlagResolutionError, match="--map-file expects one following value"):
        expand(Mapper.blueprint(), ["--map-file"])


def test_value_preserves_equals_after_the_field_name() -> None:
    assert expand(Mapper.blueprint(), ["--map-file", "a=b"]) == ["mapper.map_file=a=b"]


def test_names_come_before_flags() -> None:
    assert split_run_tokens(["go2", "nav", "--map-file", "x"]) == (
        ["go2", "nav"],
        ["--map-file", "x"],
    )


def test_at_least_one_blueprint_name_is_required() -> None:
    with pytest.raises(FlagResolutionError, match="no blueprint name given"):
        split_run_tokens(["--map-file", "x"])


def test_bare_positional_after_a_flag_errors() -> None:
    blueprint = autoconnect(Mapper.blueprint(), Planner.blueprint())

    with pytest.raises(FlagResolutionError, match="unexpected argument 'keyboard-teleop'"):
        expand(blueprint, ["--lookahead", "2", "keyboard-teleop"])


def test_explicit_option_beats_field_flag(tmp_path: Path) -> None:
    kwargs = resolve(
        Mapper.blueprint(),
        ["--map-file", "from-flag"],
        tmp_path / "config.json",
        "mapper.map_file=from-option",
    )

    assert kwargs["mapper"]["map_file"] == "from-option"


def test_field_flag_beats_config_file_and_environment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config_file = tmp_path / "config.json"
    config_file.write_text(json.dumps({"mapper": {"map_file": "from-file"}}))
    monkeypatch.setenv("MAPPER__MAP_FILE", "from-environment")

    kwargs = resolve(Mapper.blueprint(), ["--map-file", "from-flag"], config_file)

    assert kwargs["mapper"]["map_file"] == "from-flag"
