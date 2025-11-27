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

import inspect
from typing import Union, get_args, get_origin


def python_type_to_json_schema(python_type) -> dict:  # type: ignore[no-untyped-def, type-arg]
    """Convert Python type annotations to JSON Schema format."""
    # Handle None/NoneType
    if python_type is type(None) or python_type is None:
        return {"type": "null"}

    # Handle Union types (including Optional)
    origin = get_origin(python_type)
    if origin is Union:
        args = get_args(python_type)
        # Handle Optional[T] which is Union[T, None]
        if len(args) == 2 and type(None) in args:
            non_none_type = args[0] if args[1] is type(None) else args[1]
            schema = python_type_to_json_schema(non_none_type)
            # For OpenAI function calling, we don't use anyOf for optional params
            return schema
        else:
            # For other Union types, use anyOf
            return {"anyOf": [python_type_to_json_schema(arg) for arg in args]}

    # Handle List/list types
    if origin in (list, list):
        args = get_args(python_type)
        if args:
            return {"type": "array", "items": python_type_to_json_schema(args[0])}
        return {"type": "array"}

    # Handle Dict/dict types
    if origin in (dict, dict):
        return {"type": "object"}

    # Handle basic types
    type_map = {
        str: {"type": "string"},
        int: {"type": "integer"},
        float: {"type": "number"},
        bool: {"type": "boolean"},
        list: {"type": "array"},
        dict: {"type": "object"},
    }

    return type_map.get(python_type, {"type": "string"})


def function_to_schema(func) -> dict:  # type: ignore[no-untyped-def, type-arg]
    """Convert a function to OpenAI function schema format."""
    try:
        signature = inspect.signature(func)
    except ValueError as e:
        raise ValueError(f"Failed to get signature for function {func.__name__}: {e!s}")

    properties = {}
    required = []

    for param_name, param in signature.parameters.items():
        # Skip 'self' parameter for methods
        if param_name == "self":
            continue

        # Get the type annotation
        if param.annotation != inspect.Parameter.empty:
            param_schema = python_type_to_json_schema(param.annotation)
        else:
            # Default to string if no type annotation
            param_schema = {"type": "string"}

        # Add description from docstring if available (would need more sophisticated parsing)
        properties[param_name] = param_schema

        # Add to required list if no default value
        if param.default == inspect.Parameter.empty:
            required.append(param_name)

    return {
        "type": "function",
        "function": {
            "name": func.__name__,
            "description": (func.__doc__ or "").strip(),
            "parameters": {
                "type": "object",
                "properties": properties,
                "required": required,
            },
        },
    }
