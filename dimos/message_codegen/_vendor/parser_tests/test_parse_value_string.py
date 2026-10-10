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

from dimos.message_codegen._vendor.rosidl_parser import InvalidValue
from dimos.message_codegen._vendor.rosidl_parser import parse_value_string
from dimos.message_codegen._vendor.rosidl_parser import Type


def test_parse_value_string_primitive():
    parse_value_string(Type('bool'), '1')


def test_parse_value_string():
    with pytest.raises(InvalidValue):
        parse_value_string(Type('bool[]'), '1')

    with pytest.raises(InvalidValue):
        parse_value_string(Type('bool[2]'), '[1]')
    with pytest.raises(InvalidValue):
        parse_value_string(Type('bool[<=1]'), '[1, 0]')

    with pytest.raises(InvalidValue):
        parse_value_string(Type('bool[]'), '[2]')

    value = parse_value_string(Type('bool[]'), '[1]')
    assert value

    value = parse_value_string(Type('string[]'), "['foo', 'bar']")
    assert value == ['foo', 'bar']

    with pytest.raises(InvalidValue):
        parse_value_string(Type('string[<=2]'), '[foo, bar, baz]')

    with pytest.raises(InvalidValue):
        parse_value_string(Type('string[<=2]'), """["foo", 'ba"r', "ba,z"]""")

    with pytest.raises(ValueError):
        parse_value_string(Type('string[<=3]'), """[,"foo", 'ba"r', "ba,z"]""")

    with pytest.raises(ValueError):
        parse_value_string(Type('string[<=3]'), """["foo", ,'ba"r', "ba,z"]""")

    parse_value_string(Type('string[<=2]'), """["fo'o\\\", bar", baz]""")
    assert value


def test_parse_value_string_not_implemented():
    with pytest.raises(NotImplementedError):
        parse_value_string(Type('pkg/Foo[]'), '')
