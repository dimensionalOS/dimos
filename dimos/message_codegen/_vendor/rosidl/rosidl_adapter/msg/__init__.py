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
from dimos.message_codegen._vendor.rosidl.rosidl_adapter.resource import expand_template


def convert_msg_to_idl(package_dir, package_name, input_file, output_dir):
    assert package_dir.is_absolute()
    assert not input_file.is_absolute()
    assert input_file.suffix == '.msg'

    abs_input_file = package_dir / input_file
    print(f'Reading input file: {abs_input_file}')
    abs_input_file = package_dir / input_file
    content = abs_input_file.read_text(encoding='utf-8')
    msg = parse_message_string(package_name, input_file.stem, content)

    output_file = output_dir / input_file.with_suffix('.idl').name
    abs_output_file = output_file.absolute()
    print(f'Writing output file: {abs_output_file}')
    data = {
        'pkg_name': package_name,
        'relative_input_file': input_file.as_posix(),
        'msg': msg,
    }

    expand_template('msg.idl.em', data, output_file, encoding='iso-8859-1')
    return output_file


MSG_TYPE_TO_IDL = {
    'bool': 'boolean',
    'byte': 'octet',
    'char': 'uint8',
    'int8': 'int8',
    'uint8': 'uint8',
    'int16': 'int16',
    'uint16': 'uint16',
    'int32': 'int32',
    'uint32': 'uint32',
    'int64': 'int64',
    'uint64': 'uint64',
    'float32': 'float',
    'float64': 'double',
    'string': 'string',
    'wstring': 'wstring',
}


def to_idl_literal(idl_type, value):
    if idl_type[-1] == ']' or idl_type.startswith('sequence<'):
        content = repr(tuple(value)).replace('\\', r'\\').replace('"', r'\"')
        return f'"{content}"'

    if 'boolean' == idl_type:
        return 'TRUE' if value else 'FALSE'
    if idl_type.startswith('string'):
        return string_to_idl_string_literal(value)
    if idl_type.startswith('wstring'):
        return string_to_idl_wstring_literal(value)
    return value


def string_to_idl_string_literal(string):
    """Convert string to character literal as described in IDL 4.2 section  7.2.6.3 ."""
    estr = string.encode().decode('unicode_escape')
    estr = estr.replace('"', r'\"')
    return '"{0}"'.format(estr)


def string_to_idl_wstring_literal(string):
    return string_to_idl_string_literal(string)


def get_include_file(base_type):
    if base_type.is_primitive_type():
        return None
    return f'{base_type.pkg_name}/msg/{base_type.type}.idl'


def get_idl_type(type_):
    if isinstance(type_, str):
        identifier = MSG_TYPE_TO_IDL[type_]
    elif type_.is_primitive_type():
        identifier = MSG_TYPE_TO_IDL[type_.type]
        if (
            identifier in ('string', 'wstring') and
            type_.string_upper_bound is not None
        ):
            identifier += f'<{type_.string_upper_bound}>'
    else:
        identifier = f'{type_.pkg_name}::msg::{type_.type}'

    if isinstance(type_, str) or not type_.is_array:
        return identifier

    if type_.is_fixed_size_array():
        return f'{identifier}[{type_.array_size}]'

    if not type_.is_upper_bound:
        return f'sequence<{identifier}>'

    return f'sequence<{identifier}, {type_.array_size}>'
