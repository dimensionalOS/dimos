# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from dimos.message_codegen._vendor.rosidl_parser import InvalidFieldDefinition
from dimos.message_codegen._vendor.rosidl_parser import parse_message_file
from dimos.message_codegen._vendor.rosidl_parser import parse_message_string


def test_parse_message_string_with_unicode_comments():
    # Similar to `test_parse_message_string.py` but we only care about the comments part.
    msg_spec = parse_message_string('pkg', 'Foo', '#comment ڭ with ڮ some گ unicode ڰ sprinkles\
            \n \n  # ♔ ♕ ♖ ♗ ♘ ♙ ♚ ♛ ♜ ♝ ♞ ♟')
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    with pytest.raises(InvalidFieldDefinition):
        parse_message_string('pkg', 'Foo', 'bool  # comment ✌✌')


def test_parse_message_file_with_unicode_comments():
    # Like `test_parse_message_file.py` but with a unicode comment line.
    path = tempfile.mkdtemp(prefix='test_parse_message_file_with_unicode_comments_')
    try:
        filename = os.path.join(path, 'Foo.msg')
        with open(filename, 'w', encoding='utf-8') as handle:
            handle.write('bool \tfoo')
            # Adding a comment line with some unicode symbols.
            handle.write('# ∀ ∁ ∂ ∃ ∄ ∅ ∆ ∇ ∈ ∉ ∊ ∋ ∌')
        msg_spec = parse_message_file('pkg', filename)

        assert len(msg_spec.fields) == 1
        assert msg_spec.fields[0].type.type == 'bool'
        assert msg_spec.fields[0].name == 'foo'
        assert msg_spec.fields[0].default_value is None
        assert len(msg_spec.constants) == 0

        with open(filename, 'a', encoding='utf-8') as handle:
            handle.write('\nbool foo')
            handle.write('# ∀ ∁ ∂ ∃ ∄ ∅ ∆ ∇ ∈ ∉ ∊ ∋ ∌')
        with pytest.raises(ValueError) as e:
            parse_message_file('pkg', filename)
        assert 'foo' in str(e.value)
    finally:
        shutil.rmtree(path)


def test_extract_message_unicode_comments():
    # Like `test_extract_message_commnets.py` but with several unicode symbols as comments.
    # multi line file-level comment
    msg_spec = parse_message_string('pkg', 'Foo', '# ¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶\n'
                                                  '#\n# ¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×Ø\n'
                                                  'bool value')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 3
    assert msg_spec.annotations['comment'][0] == '¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶'
    assert msg_spec.annotations['comment'][1] == ''
    assert msg_spec.annotations['comment'][2] == '¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×Ø'

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert not msg_spec.fields[0].annotations['comment']

    # file-level comment separated from field-level comment
    msg_spec = parse_message_string('pkg', 'Foo', '# ɐɑɒɓɔɕɖɗɘəɚɛɜɝɞɟɠɡɢɣɤɥɦ'
                                                  '\n\n# ʙʚʛʜʝʞʟʠʡʢʣʤʥʦʧʨ\nbool value')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 1
    assert msg_spec.annotations['comment'] == ['ɐɑɒɓɔɕɖɗɘəɚɛɜɝɞɟɠɡɢɣɤɥɦ']

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == 'ʙʚʛʜʝʞʟʠʡʢʣʤʥʦʧʨ'

    # file-level comment, trailing and indented field-level comment
    msg_spec = parse_message_string(
        'pkg', 'Foo', '# ЁЂЃЄЅІЇЈЉЊЋЌЎЏАБВГДЕ\nbool value'
        '# àáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ'
        '\n   # ณดตถทธนบปผฝพฟภ\nbool value2')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 1
    assert msg_spec.annotations['comment'] == ['ЁЂЃЄЅІЇЈЉЊЋЌЎЏАБВГДЕ']

    assert len(msg_spec.fields) == 2
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 2
    assert msg_spec.fields[0].annotations['comment'][0] == 'àáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ'
    assert msg_spec.fields[0].annotations['comment'][1] == 'ณดตถทธนบปผฝพฟภ'

    assert len(msg_spec.fields[1].annotations) == 1
    assert 'comment' in msg_spec.fields[1].annotations
    assert len(msg_spec.fields[1].annotations['comment']) == 0

    # trailing field-level comment, next field-level comment
    msg_spec = parse_message_string(
        'pkg', 'Foo', 'bool value  # ἀἁἂἃἄἅἆἇἈἉἊἋἌἍἎἏἐ\n'
        '# ⁰⁴⁵⁶⁷⁸⁹⁺⁻⁼⁽⁾ⁿ₀₁₂₃₄₅₆₇₈₉₊₋₌₍₎\nbool value2')
    assert len(msg_spec.annotations) == 1
    assert 'comment' in msg_spec.annotations
    assert len(msg_spec.annotations['comment']) == 0

    assert len(msg_spec.fields) == 2
    assert len(msg_spec.fields[0].annotations) == 1
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == 'ἀἁἂἃἄἅἆἇἈἉἊἋἌἍἎἏἐ'

    assert len(msg_spec.fields[1].annotations) == 1
    assert 'comment' in msg_spec.fields[1].annotations
    assert len(msg_spec.fields[1].annotations['comment']) == 1
    assert msg_spec.fields[1].annotations['comment'][0] == '⁰⁴⁵⁶⁷⁸⁹⁺⁻⁼⁽⁾ⁿ₀₁₂₃₄₅₆₇₈₉₊₋₌₍₎'

    # field-level comment with a unit
    msg_spec = parse_message_string(
        'pkg', 'Foo', 'bool value  # ←↑→↓↔↕↖↗↘↙↚↛↜↝↞↟↠↡ [✽✾✿❀❁❂❃❄❅❆❇❈❉❊❋]')

    assert len(msg_spec.fields) == 1
    assert len(msg_spec.fields[0].annotations) == 2
    assert 'comment' in msg_spec.fields[0].annotations
    assert len(msg_spec.fields[0].annotations['comment']) == 1
    assert msg_spec.fields[0].annotations['comment'][0] == '←↑→↓↔↕↖↗↘↙↚↛↜↝↞↟↠↡'

    assert 'unit' in msg_spec.fields[0].annotations
    assert msg_spec.fields[0].annotations['unit'] == '✽✾✿❀❁❂❃❄❅❆❇❈❉❊❋'
