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

import importlib
import time

import typer

from dimos.core.transport import LCMTransport, pLCMTransport

_modules_to_try = [
    "dimos.msgs.geometry_msgs",
    "dimos.msgs.nav_msgs",
    "dimos.msgs.sensor_msgs",
    "dimos.msgs.std_msgs",
    "dimos.msgs.vision_msgs",
    "dimos.msgs.foxglove_msgs",
    "dimos.msgs.tf2_msgs",
]


def _resolve_type(type_name: str) -> type:
    for module_name in _modules_to_try:
        try:
            module = importlib.import_module(module_name)
            if hasattr(module, type_name):
                return getattr(module, type_name)  # type: ignore[no-any-return]
        except ImportError:
            continue

    raise ValueError(f"Could not find type '{type_name}' in any known message modules")


def topic_echo(topic: str, type_name: str) -> None:
    msg_type = _resolve_type(type_name)
    use_pickled = getattr(msg_type, "lcm_encode", None) is None
    transport: pLCMTransport[object] | LCMTransport[object] = (
        pLCMTransport(topic) if use_pickled else LCMTransport(topic, msg_type)
    )

    def _on_message(msg: object) -> None:
        print(msg)

    transport.subscribe(_on_message)

    typer.echo(f"Listening on {topic} for {type_name} messages... (Ctrl+C to stop)")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        typer.echo("\nStopped.")


def topic_send(topic: str, message_expr: str) -> None:
    eval_context: dict[str, object] = {}
    modules_to_import = [
        "dimos.msgs.geometry_msgs",
        "dimos.msgs.nav_msgs",
        "dimos.msgs.sensor_msgs",
        "dimos.msgs.std_msgs",
        "dimos.msgs.vision_msgs",
        "dimos.msgs.foxglove_msgs",
        "dimos.msgs.tf2_msgs",
    ]

    for module_name in modules_to_import:
        try:
            module = importlib.import_module(module_name)
            for name in getattr(module, "__all__", dir(module)):
                if not name.startswith("_"):
                    obj = getattr(module, name, None)
                    if obj is not None:
                        eval_context[name] = obj
        except ImportError:
            continue

    try:
        message = eval(message_expr, eval_context)
    except Exception as e:
        typer.echo(f"Error parsing message: {e}", err=True)
        raise typer.Exit(1)

    msg_type = type(message)
    use_pickled = getattr(msg_type, "lcm_encode", None) is None
    transport: pLCMTransport[object] | LCMTransport[object] = (
        pLCMTransport(topic) if use_pickled else LCMTransport(topic, msg_type)
    )

    transport.broadcast(None, message)  # type: ignore[arg-type]
    typer.echo(f"Sent to {topic}: {message}")
