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

import os
import shutil
import tempfile

import pytest

from dimos.message_codegen._vendor.rosidl_parser import parse_message_file


def test_parse_message_file():
    path = tempfile.mkdtemp(prefix='test_parse_message_file_')
    try:
        filename = os.path.join(path, 'Foo.msg')
        with open(filename, 'w') as handle:
            handle.write('bool \tfoo')
        msg_spec = parse_message_file('pkg', filename)

        assert len(msg_spec.fields) == 1
        assert msg_spec.fields[0].type.type == 'bool'
        assert msg_spec.fields[0].name == 'foo'
        assert msg_spec.fields[0].default_value is None
        assert len(msg_spec.constants) == 0

        with open(filename, 'a') as handle:
            handle.write('\nbool foo')
        with pytest.raises(ValueError) as e:
            parse_message_file('pkg', filename)
        assert 'foo' in str(e.value)
    finally:
        shutil.rmtree(path)
