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

"""What the gateway keeps a copy of, checked against the dimos code it copies, so the copy can't drift."""

import dataclasses
from pathlib import Path
import typing
from typing import Any

from pydantic.fields import FieldInfo
import pytest
import typer
from typer.testing import CliRunner
import yaml

from dimos.cli.commands import global_options
from dimos.cli.commands.global_options import create_dynamic_callback
from dimos.cli.dimos import normalize_argv
from dimos.core.coordination.blueprints import StreamRef
from dimos.core.global_config import GlobalConfig
from dimos.core.run_registry import RunEntry
from experimental.gateway.utils import config, models

ROOT = Path(__file__).parents[3]


def test_a_registry_run_is_a_run_entry() -> None:
    entry_fields = {field.name for field in dataclasses.fields(RunEntry)}
    # where a run on this computer came from and whether it can be stopped: the gateway's, not the registry's
    found_here = {"registry", "owner", "command", "ours", "stoppable", "whyNot"}
    assert set(models.RegistryRun.model_fields) - found_here <= entry_fields


def test_stream_directions_are_dimos_own() -> None:
    def options(annotation: object) -> set[str]:
        return set(typing.get_args(annotation))

    dimos_own = typing.get_type_hints(StreamRef)["direction"]
    assert options(models.Stream.model_fields["direction"].annotation) == options(dimos_own)


def test_dimos_yaml_version_is_the_package_version() -> None:
    found, package = config.checkout_version(ROOT)
    assert found and yaml.safe_load((ROOT / "dimos.yaml").read_text())["version"] == package


def sample(name: str, field: FieldInfo, default: Any) -> Any:
    """A value for a GlobalConfig field that isn't its default (so the flag really carries something)."""
    kind = field.annotation
    options = [a for a in typing.get_args(kind) if a is not type(None)]
    if typing.get_origin(kind) is typing.Literal:
        return next((o for o in options if o != default), default)
    if kind is bool:
        return not default
    inner = options[0] if options and typing.get_origin(kind) is not None else kind
    if typing.get_origin(inner) is typing.Literal:
        return typing.get_args(inner)[-1]
    # what Desktop sends: JSON (a Path is a string)
    samples = {str: "sample", int: 3, float: 1.5, bool: True, Path: "/tmp/sample"}
    assert inner in samples, (
        f"GlobalConfig.{name} has a type ({kind}) this test has no sample for: add one"
    )
    return samples[inner]


def test_launch_flags_are_what_dimos_reads(monkeypatch: pytest.MonkeyPatch) -> None:
    """Every GlobalConfig flag `dimos` takes, through its real root callback and argv handling, comes out as the
    value the gateway put in; and a GlobalConfig field `dimos` has no flag for is refused before a launch."""
    defaults = GlobalConfig.model_validate({}).model_dump()
    flagged = config.flag_fields()
    values = {
        name: sample(name, field, defaults[name])
        for name, field in GlobalConfig.model_fields.items()
        if name in flagged
    }
    # empty strings are a real setting too (simulation "" is off), and a bare optional-value flag means its first choice
    values["simulation"] = ""
    config.check_overrides(values)

    seen: dict[str, Any] = {}
    app = typer.Typer()
    app.callback()(create_dynamic_callback())  # type: ignore[no-untyped-call]

    @app.command()
    def run(ctx: typer.Context) -> None:
        seen.update(ctx.obj)

    @app.command()
    def other() -> None:
        pass

    monkeypatch.setattr(global_options, "global_config", GlobalConfig.model_validate({}))
    argv = normalize_argv(["dimos", *config.global_config_flags(values), "run"])[1:]
    result = CliRunner().invoke(app, argv)
    assert result.exit_code == 0, result.output
    assert set(seen) == set(values)

    def parsed(given: dict[str, Any]) -> dict[str, Any]:
        return GlobalConfig.model_validate(given).model_dump(include=set(values))

    assert parsed(seen) == parsed(values)

    unflagged = set(GlobalConfig.model_fields) - flagged
    for name in unflagged:
        with pytest.raises(ValueError, match=name):
            config.check_overrides({name: defaults[name]})
