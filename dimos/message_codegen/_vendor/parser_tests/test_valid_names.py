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

from dimos.message_codegen._vendor.rosidl_parser import InvalidResourceName
from dimos.message_codegen._vendor.rosidl_parser import is_valid_constant_name
from dimos.message_codegen._vendor.rosidl_parser import is_valid_field_name
from dimos.message_codegen._vendor.rosidl_parser import is_valid_message_name
from dimos.message_codegen._vendor.rosidl_parser import is_valid_package_name


def test_is_valid_package_name():
    for valid_package_name in [
            'foo', 'foo_bar']:
        assert is_valid_package_name(valid_package_name)
    for invalid_package_name in [
            '_foo', 'foo_', 'foo__bar', 'foo-bar']:
        assert not is_valid_package_name(invalid_package_name)
    with pytest.raises(InvalidResourceName):
        is_valid_package_name(None)


def test_is_valid_field_name():
    for valid_field_name in [
            'foo', 'foo_bar']:
        is_valid_field_name(valid_field_name)
    for invalid_field_name in [
            '_foo', 'foo_', 'foo__bar', 'foo-bar']:
        assert not is_valid_field_name(invalid_field_name)
    with pytest.raises(InvalidResourceName):
        is_valid_field_name(None)


def test_is_valid_message_name():
    for valid_message_name in [
            'Foo', 'FooBar']:
        assert is_valid_message_name(valid_message_name)
    for invalid_message_name in [
            '0foo', '_Foo', 'Foo_', 'Foo_Bar']:
        assert not is_valid_message_name(invalid_message_name)
    with pytest.raises(InvalidResourceName):
        is_valid_message_name(None)


def test_is_valid_constant_name():
    for valid_constant_name in [
            'FOO', 'FOO_BAR']:
        assert is_valid_constant_name(valid_constant_name)
    for invalid_constant_name in [
            '_FOO', 'FOO_', 'FOO__BAR', 'Foo']:
        assert not is_valid_constant_name(invalid_constant_name)
    with pytest.raises(InvalidResourceName):
        is_valid_constant_name(None)
