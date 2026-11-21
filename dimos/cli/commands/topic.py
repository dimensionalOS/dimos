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

"""`dimos topic` pub/sub commands; the implementation lives in dimos.cli.topic."""

from __future__ import annotations

import typer

topic_app = typer.Typer(help="Topic commands for pub/sub")


@topic_app.command()
def echo(
    topic: str = typer.Argument(
        ...,
        help="Topic to listen on: a name (cmd_vel or /cmd_vel) or a full channel as the logs "
        "print it (/cmd_vel#geometry_msgs.Twist on LCM, dimos/cmd_vel/geometry_msgs.Twist on Zenoh)",
    ),
    type_name: str | None = typer.Argument(
        None,
        help="Optional message type (e.g., PoseStamped). If omitted, infer it from the channel.",
    ),
) -> None:
    from dimos.cli.topic import topic_echo

    topic_echo(topic, type_name)


@topic_app.command()
def send(
    topic: str = typer.Argument(..., help="Topic to send to, in any form that echo accepts"),
    message_expr: str = typer.Argument(..., help="Python expression for the message"),
) -> None:
    from dimos.cli.topic import topic_send

    topic_send(topic, message_expr)
