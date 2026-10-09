# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from dimos.message_codegen._vendor.rosidl_parser import InvalidFieldDefinition
from dimos.message_codegen._vendor.rosidl_parser import InvalidResourceName
from dimos.message_codegen._vendor.rosidl_parser import parse_message_string


def test_parse_message_string():
    msg_spec = parse_message_string('pkg', 'Foo', '')
    assert msg_spec.base_type.pkg_name == 'pkg'
    assert msg_spec.base_type.type == 'Foo'
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    msg_spec = parse_message_string('pkg', 'Foo', '#comment\n \n  # comment')
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    with pytest.raises(InvalidFieldDefinition):
        parse_message_string('pkg', 'Foo', 'bool  # comment')

    msg_spec = parse_message_string('pkg', 'Foo', 'bool foo')
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0].type.type == 'bool'
    assert msg_spec.fields[0].name == 'foo'
    assert msg_spec.fields[0].default_value is None
    assert len(msg_spec.constants) == 0

    msg_spec = parse_message_string('pkg', 'Foo', 'bool foo 1')
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0].type.type == 'bool'
    assert msg_spec.fields[0].name == 'foo'
    assert msg_spec.fields[0].default_value
    assert len(msg_spec.constants) == 0

    with pytest.raises(InvalidResourceName):
        parse_message_string('pkg', 'Foo', 'Ty_pe foo')
    with pytest.raises(TypeError):
        parse_message_string('pkg', 'Foo', 'bool] foo')
    with pytest.raises(TypeError):
        parse_message_string('pkg', 'Foo', 'bool[max]] foo')
    with pytest.raises(ValueError) as e:
        parse_message_string('pkg', 'Foo', 'bool foo\nbool foo')
    assert 'foo' in str(e.value)

    msg_spec = parse_message_string('pkg', 'Foo', 'bool FOO=1')
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 1
    assert msg_spec.constants[0].type == 'bool'
    assert msg_spec.constants[0].name == 'FOO'
    assert msg_spec.constants[0].value

    with pytest.raises(TypeError):
        parse_message_string('pkg', 'Foo', 'pkg/Bar foo=1')
    with pytest.raises(NameError):
        parse_message_string('pkg', 'Foo', 'bool foo=1')
    with pytest.raises(ValueError) as e:
        parse_message_string('pkg', 'Foo', 'bool FOO=1\nbool FOO=1')
    assert 'FOO' in str(e.value)
