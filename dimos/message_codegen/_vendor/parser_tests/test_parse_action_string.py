# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import pytest

from dimos.message_codegen._vendor.rosidl_parser import InvalidActionSpecification
from dimos.message_codegen._vendor.rosidl_parser import parse_action_string


def test_invalid_action_specification():
    with pytest.raises(InvalidActionSpecification):
        parse_action_string('pkg', 'Foo', '')

    # too few action separators (---)
    with pytest.raises(InvalidActionSpecification):
        parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar')


def test_valid_action_string():
    parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar\n---')


def test_valid_action_string1():
    spec = parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar\n---\nbool foo')
    # Goal checks
    assert spec.goal.base_type.pkg_name == 'pkg'
    assert spec.goal.msg_name == 'Foo_Goal'
    assert len(spec.goal.fields) == 1
    assert len(spec.goal.constants) == 0
    # Result checks
    assert spec.result.base_type.pkg_name == 'pkg'
    assert spec.result.msg_name == 'Foo_Result'
    assert len(spec.result.fields) == 1
    assert len(spec.result.constants) == 0
    # Feedback checks
    assert spec.feedback.base_type.pkg_name == 'pkg'
    assert spec.feedback.msg_name == 'Foo_Feedback'
    assert len(spec.feedback.fields) == 1
    assert len(spec.feedback.constants) == 0


def test_valid_action_string2():
    spec = parse_action_string(
        'pkg', 'Foo', '#comment---\n \nbool foo\n---\n#comment\n \nint8 bar\n---\nbool foo')
    # Goal checks
    assert spec.goal.base_type.pkg_name == 'pkg'
    assert spec.goal.msg_name == 'Foo_Goal'
    assert len(spec.goal.fields) == 1
    assert len(spec.goal.constants) == 0
    # Result checks
    assert spec.result.base_type.pkg_name == 'pkg'
    assert spec.result.msg_name == 'Foo_Result'
    assert len(spec.result.fields) == 1
    assert len(spec.result.constants) == 0
    # Feedback checks
    assert len(spec.feedback.fields) == 1
    assert spec.feedback.fields[0].type.type == 'bool'
    assert spec.feedback.fields[0].name == 'foo'
    assert len(spec.feedback.constants) == 0


def test_valid_action_string3():
    spec = parse_action_string(
        'pkg',
        'Foo',
        'bool foo\nstring status\n---\nbool FOO=1\nint8 bar\n---\nbool BAR=1\nbool foo')
    # Goal checks
    assert spec.goal.base_type.pkg_name == 'pkg'
    assert spec.goal.msg_name == 'Foo_Goal'
    assert len(spec.goal.fields) == 2
    assert len(spec.goal.constants) == 0
    # Result checks
    assert spec.result.base_type.pkg_name == 'pkg'
    assert spec.result.msg_name == 'Foo_Result'
    assert len(spec.result.fields) == 1
    assert len(spec.result.constants) == 1
    assert spec.result.constants[0].type == 'bool'
    assert spec.result.constants[0].name == 'FOO'
    assert spec.result.constants[0].value
    # Feedback checks
    assert len(spec.feedback.fields) == 1
    assert spec.feedback.fields[0].type.type == 'bool'
    assert spec.feedback.fields[0].name == 'foo'
    assert len(spec.feedback.constants) == 1
    assert spec.feedback.constants[0].type == 'bool'
    assert spec.feedback.constants[0].name == 'BAR'
    assert spec.feedback.constants[0].value
