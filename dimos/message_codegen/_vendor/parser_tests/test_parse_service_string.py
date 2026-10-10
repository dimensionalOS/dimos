# Copyright 2015 Open Source Robotics Foundation, Inc.
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
from dimos.message_codegen._vendor.rosidl_parser import InvalidServiceSpecification
from dimos.message_codegen._vendor.rosidl_parser import parse_service_string


def test_parse_service_string():
    with pytest.raises(InvalidServiceSpecification):
        parse_service_string('pkg', 'Foo', '')

    srv_spec = parse_service_string('pkg', 'Foo', '---')
    assert srv_spec.pkg_name == 'pkg'
    assert srv_spec.srv_name == 'Foo'
    assert srv_spec.request.base_type.pkg_name == 'pkg'
    assert srv_spec.request.base_type.type == 'Foo_Request'
    assert len(srv_spec.request.fields) == 0
    assert len(srv_spec.request.constants) == 0
    assert srv_spec.response.base_type.pkg_name == 'pkg'
    assert srv_spec.response.base_type.type == 'Foo_Response'
    assert len(srv_spec.response.fields) == 0
    assert len(srv_spec.response.constants) == 0

    srv_spec = parse_service_string('pkg', 'Foo', '#comment\n---\n \n  # comment')
    assert len(srv_spec.request.fields) == 0
    assert len(srv_spec.request.constants) == 0
    assert len(srv_spec.response.fields) == 0
    assert len(srv_spec.response.constants) == 0

    with pytest.raises(InvalidFieldDefinition):
        parse_service_string('pkg', 'Foo', 'bool  # comment\n---')

    srv_spec = parse_service_string('pkg', 'Foo', 'bool foo\n---\nint8 bar')
    assert len(srv_spec.request.fields) == 1
    assert srv_spec.request.fields[0].type.type == 'bool'
    assert srv_spec.request.fields[0].name == 'foo'
    assert srv_spec.request.fields[0].default_value is None
    assert len(srv_spec.request.constants) == 0
    assert len(srv_spec.response.fields) == 1
    assert srv_spec.response.fields[0].type.type == 'int8'
    assert srv_spec.response.fields[0].name == 'bar'
    assert srv_spec.response.fields[0].default_value is None
    assert len(srv_spec.response.constants) == 0

    srv_spec = parse_service_string('pkg', 'Foo', 'bool foo 1\n---\nint8 bar 2')
    assert len(srv_spec.request.fields) == 1
    assert srv_spec.request.fields[0].type.type == 'bool'
    assert srv_spec.request.fields[0].name == 'foo'
    assert srv_spec.request.fields[0].default_value
    assert len(srv_spec.request.constants) == 0
    assert len(srv_spec.response.fields) == 1
    assert srv_spec.response.fields[0].type.type == 'int8'
    assert srv_spec.response.fields[0].name == 'bar'
    assert srv_spec.response.fields[0].default_value == 2
    assert len(srv_spec.response.constants) == 0

    srv_spec = parse_service_string('pkg', 'Foo', 'bool FOO=1\n---\nint8 BAR=2')
    assert len(srv_spec.request.fields) == 0
    assert len(srv_spec.request.constants) == 1
    assert srv_spec.request.constants[0].type == 'bool'
    assert srv_spec.request.constants[0].name == 'FOO'
    assert srv_spec.request.constants[0].value
    assert len(srv_spec.response.fields) == 0
    assert len(srv_spec.response.constants) == 1
    assert srv_spec.response.constants[0].type == 'int8'
    assert srv_spec.response.constants[0].name == 'BAR'
    assert srv_spec.response.constants[0].value == 2
