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
    if args.keys() != {"args"}:
        return [], args

    if first_pass:
        return interpret_tool_call_args(args["args"], first_pass=False)

    return [], args
