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

from dimos.message_codegen._vendor.rosidl_parser import Type


def test_type_constructor():
    type_ = Type('bool')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert not type_.is_array
    assert type_.array_size is None
    assert not type_.is_upper_bound

    type_ = Type('bool[]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size is None
    assert not type_.is_upper_bound

    with pytest.raises(TypeError):
        Type('bool]')

    type_ = Type('bool[5]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size == 5
    assert not type_.is_upper_bound

    type_ = Type('bool[<=5]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size == 5
    assert type_.is_upper_bound

    with pytest.raises(TypeError):
        Type('bool[size]')
    with pytest.raises(TypeError):
        Type('bool[0]')
    with pytest.raises(TypeError):
        Type('bool[<=size]')
    with pytest.raises(TypeError):
        Type('bool[<=0]')


def test_type_methods():
    assert Type('bool[5]') != 23

    assert Type('pkg/Foo') == Type('pkg/Foo')
    assert Type('pkg/Foo[]') == Type('pkg/Foo[]')
    assert Type('pkg/Foo[5]') == Type('pkg/Foo[5]')
    assert Type('pkg/Foo[<=5]') == Type('pkg/Foo[<=5]')

    assert Type('bool') != Type('pkg/Foo')
    assert Type('pkg/Foo[]') != Type('pkg/Foo[5]')
    assert Type('pkg/Foo[5]') != Type('pkg/Foo[<=5]')
    assert Type('pkg/Foo[<=5]') != Type('pkg/Foo[<=23]')

    {Type('pkg/Foo[5]'): None}

    assert str(Type('pkg/Foo[]')) == 'pkg/Foo[]'
    assert str(Type('pkg/Foo[5]')) == 'pkg/Foo[5]'
    assert str(Type('pkg/Foo[<=5]')) == 'pkg/Foo[<=5]'
