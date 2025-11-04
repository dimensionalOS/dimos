# Copyright 2025 Dimensional Inc.
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

from enum import Enum
import inspect
from typing import Optional, get_args, get_origin

import typer

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.protocol import pubsub
from dimos.robot.all_blueprints import all_blueprints, get_blueprint_by_name, get_module_by_name

RobotType = Enum("RobotType", {key.replace("-", "_").upper(): key for key in all_blueprints.keys()})

main = typer.Typer()


def create_dynamic_callback():
    fields = GlobalConfig.model_fields

    # Build the function signature dynamically
    params = [
        inspect.Parameter("ctx", inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=typer.Context),
    ]

    # Create parameters for each field in GlobalConfig
    for field_name, field_info in fields.items():
        field_type = field_info.annotation

        # Handle Optional types
        # Check for Optional/Union with None
        if get_origin(field_type) is type(Optional[str]):  # noqa: UP045
            inner_types = get_args(field_type)
            if len(inner_types) == 2 and type(None) in inner_types:
                # It's Optional[T], get the actual type T
                actual_type = next(t for t in inner_types if t != type(None))
            else:
                actual_type = field_type
        else:
            actual_type = field_type

        # Convert field name from snake_case to kebab-case for CLI
        cli_option_name = field_name.replace("_", "-")

        # Special handling for boolean fields
        if actual_type is bool:
            # For boolean fields, create --flag/--no-flag pattern
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}/--no-{cli_option_name}",
                    help=f"Override {field_name} in GlobalConfig",
                ),
                annotation=Optional[bool],  # noqa: UP045
            )
        else:
            # For non-boolean fields, use regular option
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}",
                    help=f"Override {field_name} in GlobalConfig",
                ),
                annotation=Optional[actual_type],  # noqa: UP045
            )
        params.append(param)

    def callback(**kwargs) -> None:
        ctx = kwargs.pop("ctx")
        overrides = {k: v for k, v in kwargs.items() if v is not None}
        ctx.obj = GlobalConfig().model_copy(update=overrides)

    callback.__signature__ = inspect.Signature(params)

    return callback


main.callback()(create_dynamic_callback())


@main.command()
def run(
    ctx: typer.Context,
    robot_type: RobotType = typer.Argument(..., help="Type of robot to run"),
    extra_modules: list[str] = typer.Option(
        [], "--extra-module", help="Extra modules to add to the blueprint"
    ),
) -> None:
    """Run the robot with the specified configuration."""
    config: GlobalConfig = ctx.obj
    pubsub.lcm.autoconf()
    blueprint = get_blueprint_by_name(robot_type.value)

    if extra_modules:
        loaded_modules = [get_module_by_name(mod_name) for mod_name in extra_modules]
        blueprint = autoconnect(blueprint, *loaded_modules)

    dimos = blueprint.build(global_config=config)
    dimos.loop()


@main.command()
def show_config(ctx: typer.Context) -> None:
    """Show current configuration status."""
    config: GlobalConfig = ctx.obj

    for field_name, value in config.model_dump().items():
        typer.echo(f"{field_name}: {value}")


@main.command()
def list() -> None:
    """List all available blueprints."""
    blueprints = [name for name in all_blueprints.keys() if not name.startswith("demo-")]
    for blueprint_name in sorted(blueprints):
        typer.echo(blueprint_name)


if __name__ == "__main__":
    main()
