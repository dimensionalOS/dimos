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

import codecs
import os
import re
import sys

from lark import Lark
from lark.lexer import Token
from lark.tree import pydot__tree_to_png
from lark.tree import Tree

from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import AbstractNestedType
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import AbstractType
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Action
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import ACTION_FEEDBACK_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import ACTION_GOAL_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import ACTION_RESULT_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Annotation
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Array
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import BasicType
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import BoundedSequence
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import BoundedString
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import BoundedWString
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Constant
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import CONSTANT_MODULE_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import IdlContent
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import IdlFile
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Include
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Member
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Message
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import NamedType
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import NamespacedType
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Service
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import Structure
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import UnboundedSequence
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import UnboundedString
from dimos.message_codegen._vendor.rosidl.rosidl_parser.definition import UnboundedWString

grammar_file = os.path.join(os.path.dirname(__file__), 'grammar.lark')
with open(grammar_file, mode='r', encoding='utf-8') as h:
    grammar = h.read()

_parser = None


def parse_idl_file(locator, png_file=None):
    string = locator.get_absolute_path().read_text(encoding='utf-8')
    try:
        content = parse_idl_string(string, png_file=png_file)
    except Exception as e:
        print(str(e), str(locator.get_absolute_path()), file=sys.stderr)
        raise
    return IdlFile(locator, content)


def parse_idl_string(idl_string, png_file=None):
    tree = get_ast_from_idl_string(idl_string)
    content = extract_content_from_ast(tree)

    if png_file:
        os.makedirs(os.path.dirname(png_file), exist_ok=True)
        try:
            pydot__tree_to_png(tree, png_file)
        except ImportError:
            pass

    return content


def get_ast_from_idl_string(idl_string):
    global _parser
    if _parser is None:
        _parser = Lark(grammar, start='specification', maybe_placeholders=False)
    return _parser.parse(idl_string)


def extract_content_from_ast(tree):
    content = IdlContent()

    include_directives = tree.find_data('include_directive')
    for include_directive in include_directives:
        assert len(include_directive.children) == 1
        child = include_directive.children[0]
        assert child.data in ('h_char_sequence', 'q_char_sequence')
        include_token = next(child.scan_values(_find_tokens(None)))
        content.elements.append(Include(include_token.value))

    constants = {}
    const_dcls = tree.find_data('const_dcl')
    for const_dcl in const_dcls:
        annotations = get_annotations(const_dcl)
        const_type = next(const_dcl.find_data('const_type'))
        module_identifiers = get_module_identifier_values(tree, const_dcl)
        module_comments = constants.setdefault(
            module_identifiers[-1], [])
        value = get_const_expr_value(const_dcl.children[-1])
        constant = Constant(
            get_child_identifier_value(const_dcl),
            get_abstract_type_from_const_expr(const_type, value),
            value)
        constant.annotations = annotations
        module_comments.append(constant)

    typedefs = {}
    typedef_dcls = tree.find_data('typedef_dcl')
    for typedef_dcl in typedef_dcls:
        assert len(typedef_dcl.children) == 1
        child = typedef_dcl.children[0]
        assert 'type_declarator' == child.data
        assert len(child.children) == 2
        abstract_type = get_abstract_type(child.children[0])
        child = child.children[1]
        assert 'any_declarators' == child.data
        assert len(child.children) == 1, 'Only support single typedefs atm'
        child = child.children[0]
        identifier = get_first_identifier_value(child)
        abstract_type = get_abstract_type_optionally_as_array(
            abstract_type, child)
        if identifier in typedefs:
            assert typedefs[identifier] == abstract_type
        else:
            typedefs[identifier] = abstract_type

    struct_defs = list(tree.find_data('struct_def'))
    if len(struct_defs) == 1:
        msg = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[0]),
            name=get_child_identifier_value(struct_defs[0]))))
        annotations = get_annotations(struct_defs[0])
        msg.structure.annotations += annotations
        add_message_members(msg, struct_defs[0])
        resolve_typedefed_names(msg.structure, typedefs)
        constant_module_name = msg.structure.namespaced_type.name + \
            CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            msg.constants += constants[constant_module_name]
        content.elements.append(msg)

    elif len(struct_defs) == 2:
        request = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[0]),
            name=get_child_identifier_value(struct_defs[0]))))
        assert request.structure.namespaced_type.name.endswith(
            SERVICE_REQUEST_MESSAGE_SUFFIX)
        add_message_members(request, struct_defs[0])
        resolve_typedefed_names(request.structure, typedefs)
        constant_module_name = \
            request.structure.namespaced_type.name + CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            request.constants += constants[constant_module_name]

        response = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[1]),
            name=get_child_identifier_value(struct_defs[1]))))
        assert response.structure.namespaced_type.name.endswith(
            SERVICE_RESPONSE_MESSAGE_SUFFIX)
        add_message_members(response, struct_defs[1])
        resolve_typedefed_names(response.structure, typedefs)
        constant_module_name = \
            response.structure.namespaced_type.name + CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            response.constants += constants[constant_module_name]

        assert request.structure.namespaced_type.namespaces == \
            response.structure.namespaced_type.namespaces
        request_basename = request.structure.namespaced_type.name[
            :-len(SERVICE_REQUEST_MESSAGE_SUFFIX)]
        response_basename = response.structure.namespaced_type.name[
            :-len(SERVICE_RESPONSE_MESSAGE_SUFFIX)]
        assert request_basename == response_basename

        srv = Service(
            NamespacedType(
                namespaces=request.structure.namespaced_type.namespaces,
                name=request_basename),
            request, response)
        content.elements.append(srv)

    elif len(struct_defs) == 3:
        goal = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[0]),
            name=get_child_identifier_value(struct_defs[0]))))
        assert goal.structure.namespaced_type.name.endswith(ACTION_GOAL_SUFFIX)
        add_message_members(goal, struct_defs[0])
        resolve_typedefed_names(goal.structure, typedefs)
        constant_module_name = \
            goal.structure.namespaced_type.name + CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            goal.constants += constants[constant_module_name]

        result = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[1]),
            name=get_child_identifier_value(struct_defs[1]))))
        assert result.structure.namespaced_type.name.endswith(
            ACTION_RESULT_SUFFIX)
        add_message_members(result, struct_defs[1])
        resolve_typedefed_names(result.structure, typedefs)
        constant_module_name = \
            result.structure.namespaced_type.name + CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            result.constants += constants[constant_module_name]

        assert goal.structure.namespaced_type.namespaces == \
            result.structure.namespaced_type.namespaces
        goal_basename = goal.structure.namespaced_type.name[
            :-len(ACTION_GOAL_SUFFIX)]
        result_basename = result.structure.namespaced_type.name[
            :-len(ACTION_RESULT_SUFFIX)]
        assert goal_basename == result_basename

        feedback_message = Message(Structure(NamespacedType(
            namespaces=get_module_identifier_values(tree, struct_defs[2]),
            name=get_child_identifier_value(struct_defs[2]))))
        assert feedback_message.structure.namespaced_type.name.endswith(
            ACTION_FEEDBACK_SUFFIX)
        add_message_members(feedback_message, struct_defs[2])
        resolve_typedefed_names(feedback_message.structure, typedefs)
        constant_module_name = \
            feedback_message.structure.namespaced_type.name + \
            CONSTANT_MODULE_SUFFIX
        if constant_module_name in constants:
            feedback_message.constants += constants[constant_module_name]

        action = Action(
            NamespacedType(
                namespaces=goal.structure.namespaced_type.namespaces,
                name=goal_basename),
            goal, result, feedback_message)

        all_includes = content.get_elements_of_type(Include)
        unique_include_locators = {
            include.locator for include in all_includes}
        content.elements += [
            include for include in action.implicit_includes
            if include.locator not in unique_include_locators]

        content.elements.append(action)

    else:
        assert False, \
            'Currently only .idl files with 1 (a message), 2 (a service) ' \
            'and 3 (an action) structures are supported'

    return content


def resolve_typedefed_names(structure, typedefs):
    for member in structure.members:
        type_ = member.type
        if isinstance(type_, AbstractNestedType):
            type_ = type_.value_type
        assert isinstance(type_, AbstractType)
        if isinstance(type_, NamedType):
            assert type_.name in typedefs, 'Unknown named type: ' + type_.name
            typedefed_type = typedefs[type_.name]

            # second level of indirection for arrays of structures
            if isinstance(typedefed_type, AbstractNestedType):
                if isinstance(typedefed_type.value_type, NamedType):
                    assert typedefed_type.value_type.name in typedefs, \
                        'Unknown named type: ' + typedefed_type.value_type.name
                    typedefed_type.value_type = \
                        typedefs[typedefed_type.value_type.name]

            if isinstance(member.type, AbstractNestedType):
                member.type.value_type = typedefed_type
            else:
                member.type = typedefed_type


def get_first_identifier_value(tree):
    """Get the value of the first identifier token for a node."""
    identifier_token = next(tree.scan_values(_find_tokens('IDENTIFIER')))
    return identifier_token.value


def get_child_identifier_value(tree):
    """Get the value of the first child identifier token for a node."""
    for c in tree.children:
        if not isinstance(c, Token):
            continue
        if c.type == 'IDENTIFIER':
            return c.value
    return None


def _find_tokens(token_type):
    def find(t):
        if isinstance(t, Token):
            if token_type is None or t.type == token_type:
                return t
    return find


def get_module_identifier_values(tree, target):
    """Get all module names between a tree node and a specific target node."""
    path = _find_path(tree, target)
    modules = [n for n in path if n.data == 'module_dcl']
    return [
        get_first_identifier_value(n) for n in modules]


def _find_path(node, target):
    if node == target:
        return [node]
    for c in node.children:
        if not isinstance(c, Tree):
            continue
        tail = _find_path(c, target)
        if tail is not None:
            return [node] + tail
    return None


def get_abstract_type_from_const_expr(const_expr, value):
    assert len(const_expr.children) == 1
    child = const_expr.children[0]

    if child.data in ('string_type', 'wide_string_type'):
        if 'string_type' == child.data:
            return BoundedString(len(value))
        if 'wide_string_type' == child.data:
            return BoundedWString(len(value))
        assert False

    while len(child.children) == 1:
        child = child.children[0]
    return BasicType(BASE_TYPE_SPEC_TO_IDL_TYPE[child.data])


def get_abstract_type_optionally_as_array(abstract_type, declarator):
    assert len(declarator.children) == 1
    child = declarator.children[0]
    if child.data == 'array_declarator':
        fixed_array_sizes = list(child.find_data('fixed_array_size'))
        assert len(fixed_array_sizes) == 1, \
            'Unsupported multidimensional array: ' + str(declarator)
        positive_int_const = next(
            fixed_array_sizes[0].find_data('positive_int_const'))
        size = get_positive_int_const(positive_int_const)
        abstract_type = Array(abstract_type, size)
    return abstract_type


def add_message_members(msg, tree):
    members = tree.find_data('member')
    for member in members:
        # the find_data methods seems to traverse the tree in post order
        # the highest type_spec in the subtree is therefore the last item
        type_specs = list(member.find_data('type_spec'))
        type_spec = type_specs[-1]
        abstract_type = get_abstract_type_from_type_spec(type_spec)
        declarators = member.find_data('declarator')
        annotations = get_annotations(member)
        for declarator in declarators:
            assert len(declarator.children) == 1
            child = declarator.children[0]
            if child.data == 'array_declarator':
                fixed_array_sizes = list(child.find_data('fixed_array_size'))
                assert len(fixed_array_sizes) == 1, \
                    'Unsupported multidimensional array: ' + str(member)
                positive_int_const = next(
                    fixed_array_sizes[0].find_data('positive_int_const'))
                size = get_positive_int_const(positive_int_const)
                abstract_type = Array(abstract_type, size)
            m = Member(abstract_type, get_first_identifier_value(declarator))
            m.annotations += annotations
            msg.structure.members.append(m)


BASE_TYPE_SPEC_TO_IDL_TYPE = {
    'floating_pt_type_float': 'float',
    'floating_pt_type_double': 'double',
    'floating_pt_type_long_double': 'long double',
    'char_type': 'char',
    'wide_char_type': 'wchar',
    'boolean_type': 'boolean',
    'octet_type': 'octet',
    'signed_tiny_int': 'int8',
    'unsigned_tiny_int': 'uint8',
    'signed_short_int': 'int16',
    'unsigned_short_int': 'uint16',
    'signed_long_int': 'int32',
    'unsigned_long_int': 'uint32',
    'signed_longlong_int': 'int64',
    'unsigned_longlong_int': 'uint64',
}


def get_abstract_type_from_type_spec(type_spec):
    assert len(type_spec.children) == 1
    child = type_spec.children[0]
    return get_abstract_type(child)


def get_abstract_type(tree):
    if 'simple_type_spec' == tree.data:
        assert len(tree.children) == 1
        child = tree.children[0]

        if 'base_type_spec' == child.data:
            while len(child.children) == 1:
                child = child.children[0]
            return BasicType(BASE_TYPE_SPEC_TO_IDL_TYPE[child.data])

        if 'scoped_name' == child.data:
            scoped_name_separators = list(
                child.find_data('scoped_name_separator'))
            if not scoped_name_separators:
                return NamedType(get_first_identifier_value(child))
            identifiers = list(child.scan_values(_find_tokens('IDENTIFIER')))
            assert len(identifiers) > 1
            return NamespacedType(identifiers[:-1], identifiers[-1])

        assert False, 'Unsupported tree: ' + str(child)

    if 'template_type_spec' == tree.data:
        assert len(tree.children) == 1
        child = tree.children[0]

        if 'sequence_type' == child.data:
            # the find_data methods seems to traverse the tree in post order
            # the highest type_spec in the subtree is therefore the last item
            type_specs = list(child.find_data('type_spec'))
            type_spec = type_specs[-1]
            basetype = get_abstract_type_from_type_spec(type_spec)
            positive_int_consts = list(child.find_data('positive_int_const'))
            if positive_int_consts:
                path = _find_path(child, positive_int_consts[0])
                if len(path) > 2 and path[-2].data == 'string_type':
                    positive_int_consts.pop(0)
            if positive_int_consts:
                maximum_size = get_positive_int_const(positive_int_consts[-1])
                return BoundedSequence(basetype, maximum_size)
            else:
                return UnboundedSequence(basetype)

        if child.data in ('string_type', 'wide_string_type'):
            if len(child.children) == 1:
                assert child.children[0].data == 'positive_int_const'
                maximum_size = get_positive_int_const(child.children[0])
                if 'string_type' == child.data:
                    assert maximum_size > 0
                    return BoundedString(maximum_size=maximum_size)
                if 'wide_string_type' == child.data:
                    return BoundedWString(maximum_size=maximum_size)
            else:
                if 'string_type' == child.data:
                    return UnboundedString()
                if 'wide_string_type' == child.data:
                    return UnboundedWString()

        if 'fixed_pt_type' == child.data:
            assert False, 'TODO'

        assert False, 'Unsupported tree: ' + str(child)

    assert False, 'Unsupported tree: ' + str(tree)


def get_positive_int_const(positive_int_const):
    assert positive_int_const.data == 'positive_int_const'
    # TODO support arbitrary expressions
    try:
        decimal_literal = next(positive_int_const.find_data('decimal_literal'))
    except StopIteration:
        pass
    else:
        digits = ''
        for child in decimal_literal.children:
            digits += child.value
        return int(digits)

    try:
        identifier_token = next(
            positive_int_const.scan_values(_find_tokens('IDENTIFIER')))
    except StopIteration:
        pass
    else:
        # TODO ensure that identifier resolves to a positive integer
        return identifier_token.value

    assert False, 'Unsupported tree: ' + str(positive_int_const)


def get_annotations(tree):
    annotations = []
    for c in tree.children:
        if not isinstance(c, Tree):
            continue
        if c.data != 'annotation_appl':
            continue
        annotation_appl = c
        params = list(annotation_appl.find_data('annotation_appl_param'))
        if params:
            value = {}
            for param in params:
                const_expr = next(param.find_data('const_expr'))
                value[get_first_identifier_value(param)] = \
                    get_const_expr_value(const_expr)
        elif len(annotation_appl.children) == 1:
            value = None
        else:
            const_expr = next(annotation_appl.find_data('const_expr'))
            value = get_const_expr_value(const_expr)
        annotations.append(
            Annotation(get_first_identifier_value(annotation_appl), value))

    return annotations


def get_const_expr_value(const_expr):
    # TODO support arbitrary expressions
    expr = list(const_expr.find_data('primary_expr'))
    assert len(expr) == 1, str(expr)
    primary_expr = expr[0]
    assert len(primary_expr.children) == 1
    child = primary_expr.children[0]
    if 'scoped_name' == child.data:
        return str(child.children[0])
    elif 'literal' == child.data:
        literal = child
        unary_operator_minuses = list(const_expr.find_data('unary_operator_minus'))
        negate_value = len(unary_operator_minuses) % 2

        assert len(literal.children) == 1
        child = literal.children[0]

        if child.data == 'integer_literal':
            assert len(child.children) == 1
            child = child.children[0]

            if child.data == 'decimal_literal':
                value = get_decimal_literal_value(child)
                if negate_value:
                    value = -value
                return value

            assert False, 'Unsupported tree: ' + str(child)

        if child.data == 'floating_pt_literal':
            value = get_floating_pt_literal_value(child)
            if negate_value:
                value = -value
            return value

        if child.data == 'fixed_pt_literal':
            value = get_fixed_pt_literal_value(child)
            if negate_value:
                value = -value
            return value

        if child.data == 'boolean_literal':
            assert len(child.children) == 1
            child = child.children[0]
            assert child.data in ('boolean_literal_true', 'boolean_literal_false')
            return child.data == 'boolean_literal_true'

        if child.data == 'string_literals':
            assert not negate_value
            return get_string_literals_value(child, allow_unicode=False)

        if child.data == 'wide_string_literals':
            assert not negate_value
            return get_string_literals_value(child, allow_unicode=True)

        assert False, 'Unsupported tree: ' + str(const_expr)
    else:
        assert False, 'Unsupported tree: ' + str(const_expr)


def get_decimal_literal_value(decimal_literal):
    value = ''
    for child in decimal_literal.children:
        value += child.value
    return int(value)


def get_floating_pt_literal_value(floating_pt_literal):
    value = ''
    for child in floating_pt_literal.children:
        if isinstance(child, Token):
            value += child.value
        else:
            assert False, 'Unsupported tree: ' + str(floating_pt_literal)
    return float(value)


def get_fixed_pt_literal_value(fixed_pt_literal):
    value = ''
    for child in fixed_pt_literal.children:
        if isinstance(child, Token):
            value += child.value
        else:
            assert False, 'Unsupported tree: ' + str(fixed_pt_literal)
    return float(value)


def get_string_literals_value(string_literals, *, allow_unicode=False):
    assert len(string_literals.children) > 0
    value = ''
    for string_literal in string_literals.children:
        value += get_string_literal_value(
            string_literal, allow_unicode=allow_unicode)
    return value


def get_string_literal_value(string_literal, *, allow_unicode=False):
    if len(string_literal.children) == 0:
        return ''
    assert len(string_literal.children) == 1
    child = string_literal.children[0]
    assert isinstance(child, Token)
    value = child.value

    assert child.type in ('ESCAPED_STRING', 'ESCAPED_WIDE_STRING')
    if 'ESCAPED_WIDE_STRING' == child.type:
        assert len(value) >= 3
        # Get rid of leading L" and trailing "
        value = value[2:-1]
    else:
        assert len(value) >= 2
        # Get rid of leading " and trailing "
        value = value[1:-1]

    regex = _get_escape_sequences_regex(allow_unicode=allow_unicode)
    value = regex.sub(_decode_escape_sequence, value)
    # unescape double quote and backslash if preceeded by a backslash
    i = 0
    while i < len(value):
        if value[i] == '\\':
            if i + 1 < len(value) and value[i + 1] in ('"', '\\'):
                value = value[:i] + value[i + 1:]
        i += 1
    return value


def _get_escape_sequences_regex(*, allow_unicode):
    # IDL Table 7-9: Escape sequences
    pattern = '('
    # newline, horizontal tab, vertical tab, backspace, carriage return,
    # form feed, alert, backslash, question mark, single quote, double quote
    pattern += r'\\[ntvbrfa\\?\'"]'
    # octal number
    pattern += '|' + r'\\[0-7]{1,3}'
    # hexadecimal number
    pattern += '|' + r'\\x[0-9a-fA-F]{1,2}'
    if allow_unicode:
        # unicode character
        pattern += '|' + r'\\u[0-9a-fA-F]{1,4}'
    pattern += ')'

    return re.compile(pattern)


def _decode_escape_sequence(match):
    return codecs.decode(match.group(0), 'unicode-escape')
