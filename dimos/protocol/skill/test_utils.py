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

from dimos.protocol.skill.utils import interpret_tool_call_args


def test_list() -> None:
    args, kwargs = interpret_tool_call_args([1, 2, 3])
    assert args == [1, 2, 3]
    assert kwargs == {}


def test_none() -> None:
    args, kwargs = interpret_tool_call_args(None)
    assert args == []
    assert kwargs == {}


def test_none_nested() -> None:
    args, kwargs = interpret_tool_call_args({"args": None})
    assert args == []
    assert kwargs == {}


def test_non_dict() -> None:
    args, kwargs = interpret_tool_call_args("test")
    assert args == ["test"]
    assert kwargs == {}


def test_dict_with_args_and_kwargs() -> None:
    args, kwargs = interpret_tool_call_args({"args": [1, 2], "kwargs": {"key": "value"}})
    assert args == [1, 2]
    assert kwargs == {"key": "value"}


def test_dict_with_only_kwargs() -> None:
    args, kwargs = interpret_tool_call_args({"kwargs": {"a": 1, "b": 2}})
    assert args == []
    assert kwargs == {"a": 1, "b": 2}


def test_dict_as_kwargs() -> None:
    args, kwargs = interpret_tool_call_args({"x": 10, "y": 20})
    assert args == []
    assert kwargs == {"x": 10, "y": 20}


def test_dict_with_only_args_first_pass() -> None:
    args, kwargs = interpret_tool_call_args({"args": [5, 6, 7]})
    assert args == [5, 6, 7]
    assert kwargs == {}


def test_dict_with_only_args_nested() -> None:
    args, kwargs = interpret_tool_call_args({"args": {"inner": "value"}})
    assert args == []
    assert kwargs == {"inner": "value"}


def test_empty_list() -> None:
    args, kwargs = interpret_tool_call_args([])
    assert args == []
    assert kwargs == {}


def test_empty_dict() -> None:
    args, kwargs = interpret_tool_call_args({})
    assert args == []
    assert kwargs == {}


def test_integer() -> None:
    args, kwargs = interpret_tool_call_args(42)
    assert args == [42]
    assert kwargs == {}
