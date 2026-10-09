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

from dimos.message_codegen._vendor.rosidl_parser import Constant
from dimos.message_codegen._vendor.rosidl_parser import Field
from dimos.message_codegen._vendor.rosidl_parser import MessageSpecification
from dimos.message_codegen._vendor.rosidl_parser import Type


def test_message_specification_constructor():
    msg_spec = MessageSpecification('pkg', 'Foo', [], [])
    assert msg_spec.base_type.pkg_name == 'pkg'
    assert msg_spec.base_type.type == 'Foo'
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    with pytest.raises(TypeError):
        MessageSpecification('pkg', 'Foo', None, [])
    with pytest.raises(TypeError):
        MessageSpecification('pkg', 'Foo', [], None)
    with pytest.raises(TypeError):
        MessageSpecification('pkg', 'Foo', ['field'], [])
    with pytest.raises(TypeError):
        MessageSpecification('pkg', 'Foo', [], ['constant'])

    field = Field(Type('bool'), 'foo', '1')
    constant = Constant('bool', 'BAR', '1')
    msg_spec = MessageSpecification('pkg', 'Foo', [field], [constant])
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0] == field
    assert len(msg_spec.constants) == 1
    assert msg_spec.constants[0] == constant

    with pytest.raises(ValueError):
        MessageSpecification('pkg', 'Foo', [field, field], [])
    with pytest.raises(ValueError):
        MessageSpecification('pkg', 'Foo', [], [constant, constant])


def test_message_specification_methods():
    field = Field(Type('bool'), 'foo', '1')
    constant = Constant('bool', 'BAR', '1')
    msg_spec = MessageSpecification('pkg', 'Foo', [field], [constant])

    assert msg_spec != 'spec'
    assert msg_spec == MessageSpecification('pkg', 'Foo', [field], [constant])
    assert msg_spec != MessageSpecification('pkg', 'Bar', [], [])
    assert msg_spec != MessageSpecification('pkg', 'Foo', [field], [])
    assert msg_spec != MessageSpecification('pkg', 'Foo', [], [constant])
