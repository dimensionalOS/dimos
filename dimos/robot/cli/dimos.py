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
import sys
from typing import Any, Optional, get_args, get_origin

import typer

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.protocol import pubsub
from dimos.robot.all_blueprints import all_blueprints, get_blueprint_by_name, get_module_by_name
from dimos.robot.cli.topic import topic_echo, topic_send
from dimos.utils.logging_config import setup_exception_handler

RobotType = Enum("RobotType", {key.replace("-", "_").upper(): key for key in all_blueprints.keys()})  # type: ignore[misc]

main = typer.Typer(
    help="Dimensional CLI",
    no_args_is_help=True,
)


def create_dynamic_callback():  # type: ignore[no-untyped-def]
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

    def callback(**kwargs) -> None:  # type: ignore[no-untyped-def]
        ctx = kwargs.pop("ctx")
        ctx.obj = {k: v for k, v in kwargs.items() if v is not None}

    callback.__signature__ = inspect.Signature(params)  # type: ignore[attr-defined]

    return callback


main.callback()(create_dynamic_callback())  # type: ignore[no-untyped-call]


@main.command()
def run(
    ctx: typer.Context,
    robot_type: RobotType = typer.Argument(..., help="Type of robot to run"),
    extra_modules: list[str] = typer.Option(  # type: ignore[valid-type]
        [], "--extra-module", help="Extra modules to add to the blueprint"
    ),
) -> None:
    """Start a robot blueprint"""
    setup_exception_handler()

    cli_config_overrides: dict[str, Any] = ctx.obj
    pubsub.lcm.autoconf()  # type: ignore[attr-defined]
    blueprint = get_blueprint_by_name(robot_type.value)

    if extra_modules:
        loaded_modules = [get_module_by_name(mod_name) for mod_name in extra_modules]  # type: ignore[attr-defined]
        blueprint = autoconnect(blueprint, *loaded_modules)

    dimos = blueprint.build(cli_config_overrides=cli_config_overrides)
    dimos.loop()


@main.command()
def show_config(ctx: typer.Context) -> None:
    """Show current config settings and their values."""
    cli_config_overrides: dict[str, Any] = ctx.obj
    config = GlobalConfig().model_copy(update=cli_config_overrides)

    for field_name, value in config.model_dump().items():
        typer.echo(f"{field_name}: {value}")


@main.command()
def list() -> None:
    """List all available blueprints."""
    blueprints = [name for name in all_blueprints.keys() if not name.startswith("demo-")]
    for blueprint_name in sorted(blueprints):
        typer.echo(blueprint_name)


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def lcmspy(ctx: typer.Context) -> None:
    """LCM spy tool for monitoring LCM messages."""
    from dimos.utils.cli.lcmspy.run_lcmspy import main as lcmspy_main

    sys.argv = ["lcmspy", *ctx.args]
    lcmspy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def skillspy(ctx: typer.Context) -> None:
    """Skills spy tool for monitoring skills."""
    from dimos.utils.cli.skillspy.skillspy import main as skillspy_main

    sys.argv = ["skillspy", *ctx.args]
    skillspy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def agentspy(ctx: typer.Context) -> None:
    """Agent spy tool for monitoring agents."""
    from dimos.utils.cli.agentspy.agentspy import main as agentspy_main

    sys.argv = ["agentspy", *ctx.args]
    agentspy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def humancli(ctx: typer.Context) -> None:
    """Interface interacting with agents."""
    from dimos.utils.cli.human.humanclianim import main as humancli_main

    sys.argv = ["humancli", *ctx.args]
    humancli_main()


topic_app = typer.Typer(help="Topic commands for pub/sub")
main.add_typer(topic_app, name="topic")


@topic_app.command()
def echo(
    topic: str = typer.Argument(..., help="Topic name to listen on (e.g., /goal_request)"),
    type_name: str = typer.Argument(..., help="Message type (e.g., PoseStamped)"),
) -> None:
    topic_echo(topic, type_name)


@topic_app.command()
def send(
    topic: str = typer.Argument(..., help="Topic name to send to (e.g., /goal_request)"),
    message_expr: str = typer.Argument(..., help="Python expression for the message"),
) -> None:
    topic_send(topic, message_expr)


if __name__ == "__main__":
    main()
