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


def test_constant_constructor():
    value = Constant('bool', 'FOO', '1')
    assert value

    with pytest.raises(TypeError):
        Constant('pkg/Foo', 'FOO', '')

    with pytest.raises(NameError):
        Constant('bool', 'FOO BAR', '')

    with pytest.raises(ValueError):
        Constant('bool', 'FOO', None)


def test_constant_methods():
    assert Constant('bool', 'FOO', '1') != 23

    assert Constant('bool', 'FOO', '1') == Constant('bool', 'FOO', '1')
    assert Constant('bool', 'FOO', '1') != Constant('bool', 'FOO', '0')
    assert Constant('bool', 'FOO', '1') != Constant('bool', 'BAR', '1')
    assert Constant('bool', 'FOO', '1') != Constant('byte', 'FOO', '1')

    assert str(Constant('bool', 'FOO', '1')) == 'bool FOO=True'

    assert str(Constant('string', 'FOO', 'foo')) == "string FOO='foo'"
    assert str(Constant('wstring', 'FOO', 'foo')) == "wstring FOO='foo'"
