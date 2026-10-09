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

from dimos.message_codegen._vendor.rosidl_parser import Field
from dimos.message_codegen._vendor.rosidl_parser import InvalidValue
from dimos.message_codegen._vendor.rosidl_parser import Type


def test_field_constructor():
    type_ = Type('bool')
    field = Field(type_, 'foo')
    assert field.type == type_
    assert field.name == 'foo'
    assert field.default_value is None

    field = Field(type_, 'foo', '1')
    assert field.default_value

    with pytest.raises(TypeError):
        Field('type', 'foo')

    with pytest.raises(NameError):
        Field(type_, 'foo bar')

    type_ = Type('bool[2]')
    field = Field(type_, 'foo', '[false, true]')
    assert field.default_value == [False, True]

    type_ = Type('bool[]')
    field = Field(type_, 'foo', '[false, true, false]')
    assert field.default_value == [False, True, False]

    type_ = Type('bool[3]')
    with pytest.raises(InvalidValue):
        Field(type_, 'foo', '[false, true]')


def test_field_methods():
    assert Field(Type('bool'), 'foo') != 23

    assert (Field(Type('bool'), 'foo', '1') ==
            Field(Type('bool'), 'foo', 'true'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('bool'), 'foo', 'false'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('bool'), 'bar', '1'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('byte'), 'foo', '1'))

    assert str(Field(Type('bool'), 'foo', '1')) == 'bool foo True'

    assert str(Field(Type('string<=5'), 'foo', 'value')) == \
        "string<=5 foo 'value'"
