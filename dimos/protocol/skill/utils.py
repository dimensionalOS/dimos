# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any


def interpret_tool_call_args(
    args: Any, first_pass: bool = True
) -> tuple[list[Any], dict[str, Any]]:
    """
    Agents sometimes produce bizarre calls. This tries to interpret the args better.
    """

    if isinstance(args, list):
        return args, {}
    if args is None:
        return [], {}
    if not isinstance(args, dict):
        return [args], {}
    if args.keys() == {"args", "kwargs"}:
        return args["args"], args["kwargs"]
    if args.keys() == {"kwargs"}:
        return [], args["kwargs"]

    # Check if all keys are numeric strings (e.g., {'0': 'value', '1': 'value2'})
    # This happens when the agent returns positional args as a dict with index keys
    if args and all(key.isdigit() for key in args.keys()):
        # Convert to positional args list, sorted by index
        sorted_items = sorted(args.items(), key=lambda x: int(x[0]))
        return [v for _, v in sorted_items], {}

    if args.keys() != {"args"}:
        return [], args

    if first_pass:
        return interpret_tool_call_args(args["args"], first_pass=False)

    return [], args
