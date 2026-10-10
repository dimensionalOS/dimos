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

from dimos.message_codegen._vendor.rosidl_parser import parse_message_string


def test_extract_message_comments():
    # multi line file-level comment
    msg_spec = parse_message_string('pkg', 'Foo', '# comment 1\n#\n# comment 2\nbool value')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 3
    assert msg_spec.annotations['comment'][0] == 'comment 1'
    assert msg_spec.annotations['comment'][1] == ''
    assert msg_spec.annotations['comment'][2] == 'comment 2'

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert not msg_spec.fields[0].annotations['comment']

    # file-level comment separated from field-level comment
    msg_spec = parse_message_string('pkg', 'Foo', '# comment 1\n\n# comment 2\nbool value')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 1
    assert msg_spec.annotations['comment'] == ['comment 1']

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == 'comment 2'

    # file-level comment, trailing and indented field-level comment
    msg_spec = parse_message_string(
        'pkg', 'Foo', '# comment 1\nbool value  # comment 2\n   # comment 3\nbool value2')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 1
    assert msg_spec.annotations['comment'] == ['comment 1']

    assert len(msg_spec.fields) == 2
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 2
    assert msg_spec.fields[0].annotations['comment'][0] == 'comment 2'
    assert msg_spec.fields[0].annotations['comment'][1] == 'comment 3'

    assert len(msg_spec.fields[1].annotations) == 1
    assert 'comment' in msg_spec.fields[1].annotations
    assert len(msg_spec.fields[1].annotations['comment']) == 0

    # trailing field-level comment, next field-level comment
    msg_spec = parse_message_string(
        'pkg', 'Foo', 'bool value  # comment 2\n# comment 3\nbool value2')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 0

    assert len(msg_spec.fields) == 2
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == 'comment 2'

    assert len(msg_spec.fields[1].annotations) == 1
    assert 'comment' in msg_spec.fields[1].annotations
    assert len(msg_spec.fields[1].annotations['comment']) == 1
    assert msg_spec.fields[1].annotations['comment'][0] == 'comment 3'

    # field-level comment with a unit
    msg_spec = parse_message_string(
        'pkg', 'Foo', 'bool value  # comment [unit]')

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 2
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == 'comment'

    assert 'unit' in msg_spec.fields[0].annotations
    assert msg_spec.fields[0].annotations['unit'] == 'unit'
