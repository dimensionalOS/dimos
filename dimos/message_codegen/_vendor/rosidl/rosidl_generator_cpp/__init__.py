# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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

from ast import literal_eval

from ..rosidl_parser.definition import AbstractGenericString
from ..rosidl_parser.definition import AbstractNestedType
from ..rosidl_parser.definition import AbstractSequence
from ..rosidl_parser.definition import AbstractString
from ..rosidl_parser.definition import AbstractWString
from ..rosidl_parser.definition import Array
from ..rosidl_parser.definition import BasicType
from ..rosidl_parser.definition import BoundedSequence
from ..rosidl_parser.definition import FLOATING_POINT_TYPES
from ..rosidl_parser.definition import NamespacedType
from ..rosidl_parser.definition import UnboundedSequence
from ..rosidl_pycommon import generate_files


def generate_cpp(generator_arguments_file):
    mapping = {
        'idl.hpp.em': '%s.hpp',
        'idl__builder.hpp.em': 'detail/%s__builder.hpp',
        'idl__struct.hpp.em': 'detail/%s__struct.hpp',
        'idl__traits.hpp.em': 'detail/%s__traits.hpp',
        'idl__type_support.hpp.em': 'detail/%s__type_support.hpp',
    }
    return generate_files(
        generator_arguments_file, mapping,
        post_process_callback=prefix_with_bom_if_necessary)


def prefix_with_bom_if_necessary(content):
    try:
        content.encode('ASCII')
    except UnicodeError:
        prefix = '\ufeff' + \
            '// NOLINT: This file starts with a BOM ' + \
            'since it contain non-ASCII characters\n'
        content = prefix + content
    return content


MSG_TYPE_TO_CPP = {
    'boolean': 'bool',
    'octet': 'unsigned char',  # TODO change to std::byte with C++17
    'char': 'unsigned char',
    'wchar': 'char16_t',
    'float': 'float',
    'double': 'double',
    'long double': 'long double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': 'std::basic_string<char, std::char_traits<char>, ' +
              'typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>',
    'wstring': 'std::basic_string<char16_t, std::char_traits<char16_t>, typename ' +
               'std::allocator_traits<ContainerAllocator>::template rebind_alloc<char16_t>>',
}


def msg_type_only_to_cpp(type_):
    """
    Convert a message type into the C++ declaration, ignoring array types.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, BasicType):
        cpp_type = MSG_TYPE_TO_CPP[type_.typename]
    elif isinstance(type_, AbstractString):
        cpp_type = MSG_TYPE_TO_CPP['string']
    elif isinstance(type_, AbstractWString):
        cpp_type = MSG_TYPE_TO_CPP['wstring']
    elif isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaced_name())
        cpp_type = typename + '_<ContainerAllocator>'
    else:
        assert False, type_

    return cpp_type


def msg_type_to_cpp(type_):
    """
    Convert a message type into the C++ declaration, along with the array type.

    Example input: uint32, std_msgs/String, std_msgs/String[3]
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>,
                    std::array<std_msgs::String_<ContainerAllocator>, 3>

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    cpp_type = msg_type_only_to_cpp(type_)

    if isinstance(type_, AbstractNestedType):
        if isinstance(type_, UnboundedSequence):
            return \
                ('std::vector<%s, typename std::allocator_traits<ContainerAllocator>::template ' +
                 'rebind_alloc<%s>>') % (cpp_type, cpp_type)
        elif isinstance(type_, BoundedSequence):
            return \
                ('rosidl_runtime_cpp::BoundedVector<%s, %u, typename std::allocator_traits' +
                 '<ContainerAllocator>::template rebind_alloc<%s>>') % (cpp_type,
                                                                        type_.maximum_size,
                                                                        cpp_type)
        else:
            assert isinstance(type_, Array)
            return 'std::array<%s, %u>' % (cpp_type, type_.size)
    else:
        return cpp_type


def value_to_cpp(type_, value):
    """
    Convert a python value into a string representing that value in C++.

    This is equivalent to primitive_value_to_cpp but can process arrays values as well

    Warning this still processes only primitive types
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float, str or list)
    @returns: a string containing the C++ representation of the value
    """
    assert not isinstance(type_, NamespacedType), \
        "Could not convert non-primitive type '%s' to CPP" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if not isinstance(type_, AbstractNestedType):
        return primitive_value_to_cpp(type_, value)

    cpp_values = []
    is_string_array = isinstance(type_.value_type, AbstractGenericString)
    for single_value in value:
        cpp_value = primitive_value_to_cpp(type_.value_type, single_value)
        if is_string_array:
            tmp_cpp_value = '{%s}' % cpp_value
        else:
            tmp_cpp_value = cpp_value
        cpp_values.append(tmp_cpp_value)
    cpp_value = '{%s}' % ', '.join(cpp_values)
    if len(cpp_values) > 1 and not is_string_array:
        # Only wrap in a second set of {} if the array length is > 1.
        # This avoids "warning: braces around scalar initializer"
        cpp_value = '{%s}' % cpp_value
    return cpp_value


def primitive_value_to_cpp(type_, value):
    """
    Convert a python value into a string representing that value in C++.

    Warning: The value has to be a primitive and not a list
      (aka this function doesn't work for arrays)
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float or str)
    @returns: a string containing the C++ representation of the value
    """
    assert isinstance(type_, (BasicType, AbstractGenericString)), \
        "Could not convert non-primitive type '%s' to CPP" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if isinstance(type_, AbstractString):
        return '"%s"' % escape_string(value)

    if isinstance(type_, AbstractWString):
        return 'u"%s"' % escape_wstring(value)

    if type_.typename == 'boolean':
        return 'true' if value else 'false'

    if type_.typename in [
        'short', 'unsigned short',
        'char', 'wchar',
        'double', 'long double',
        'octet',
        'int8', 'uint8',
        'int16', 'uint16',
    ]:
        return str(value)

    if type_.typename == 'int32':
        # Handle edge case for INT32_MIN
        # Specifically, MSVC is not happy in this case
        if -2147483648 == value:
            return '({0}l - 1)'.format(value + 1)
        return '%sl' % value

    if type_.typename == 'uint32':
        return '%sul' % value

    if type_.typename == 'int64':
        # Handle edge case for INT64_MIN
        # See https://en.cppreference.com/w/cpp/language/integer_literal
        if -9223372036854775808 == value:
            return '(%sll - 1)' % (value + 1)
        return '%sll' % value

    if type_.typename == 'uint64':
        return '%sull' % value

    if type_.typename == 'float':
        return '%sf' % value

    assert False, "unknown primitive type '%s'" % type_.typename


def default_value_from_type(type_):
    if isinstance(type_, AbstractGenericString):
        return ''
    elif isinstance(type_, BasicType) and type_.typename in FLOATING_POINT_TYPES:
        return 0.0
    elif isinstance(type_, BasicType) and type_.typename == 'boolean':
        return False
    return 0


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s


def escape_wstring(s):
    return escape_string(s)


def create_init_alloc_and_member_lists(message):
    # A Member object represents the information we need to know to initialize
    # a single member of the class.
    class Member:

        def __init__(self, name):
            self.name = name
            self.default_value = None
            self.zero_value = None
            self.zero_need_array_override = False
            self.type = None
            self.num_prealloc = 0

        def same_default_and_zero_value(self, other):
            return self.default_value == other.default_value and \
                self.zero_value == other.zero_value

    # A CommonMemberSet is a set of adjacent members that share the same set of
    # initialization semantics.  Here, initialization semantics mean that all
    # members of the set have a default value (or do not have a default value),
    # and all members of the set have a zero value (or do not have a zero
    # value).
    class CommonMemberSet:

        def __init__(self):
            self.members = []

        def add_member(self, member):
            if not self.members or self.members[-1].same_default_and_zero_value(member):
                self.members.append(member)
                return True
            return False

    # The loop below is used to generate three different lists:
    #   init_list - The list of member variables that we will initialize using member
    #               initialization in the default constructor
    #   alloc_list - The list of member variables that we will initialize using member
    #                initializion in the allocator constructor
    #   member_list - The list of members that we will generate initialization code
    #                 for in the body of the constructors
    init_list = []
    alloc_list = []
    member_list = []
    for field in message.structure.members:
        member = Member(field.name)
        member.type = field.type
        if isinstance(field.type, Array):
            alloc_list.append(field.name + '(_alloc)')
            if isinstance(field.type.value_type, BasicType) or \
                    isinstance(field.type.value_type, AbstractGenericString):
                default = default_value_from_type(field.type.value_type)
                single = primitive_value_to_cpp(field.type.value_type, default)
                member.zero_value = [single] * field.type.size
                if field.has_annotation('default'):
                    default_value = literal_eval(
                        field.get_annotation_value('default')['value'])
                    member.default_value = []
                    for val in default_value:
                        member.default_value.append(
                            primitive_value_to_cpp(field.type.value_type, val))
            else:
                member.zero_value = []
                member.zero_need_array_override = True
        elif isinstance(field.type, AbstractSequence):
            if field.has_annotation('default'):
                default_value = literal_eval(
                    field.get_annotation_value('default')['value'])
                member.default_value = value_to_cpp(field.type, default_value)
                member.num_prealloc = len(default_value)
        else:
            if isinstance(field.type, BasicType) or \
                    isinstance(field.type, AbstractGenericString):
                if isinstance(field.type, AbstractGenericString):
                    alloc_list.append(field.name + '(_alloc)')
                default = default_value_from_type(field.type)
                member.zero_value = primitive_value_to_cpp(field.type, default)
                if field.has_annotation('default'):
                    member.default_value = primitive_value_to_cpp(
                        field.type,
                        field.get_annotation_value('default')['value'])
            else:
                init_list.append(field.name + '(_init)')
                alloc_list.append(field.name + '(_alloc, _init)')

        if field.has_annotation('default') or member.zero_value is not None:
            if not member_list or not member_list[-1].add_member(member):
                commonset = CommonMemberSet()
                commonset.add_member(member)
                member_list.append(commonset)

    return init_list, alloc_list, member_list
