@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
@{
from .rosidl_generator_cpp.api import create_init_alloc_and_member_lists
from .rosidl_generator_cpp.api import escape_string
from .rosidl_generator_cpp.api import escape_wstring
from .rosidl_generator_cpp.api import msg_type_to_cpp
from .rosidl_generator_cpp.api import MSG_TYPE_TO_CPP
from .rosidl_parser.definition import AbstractNestedType
from .rosidl_parser.definition import AbstractString
from .rosidl_parser.definition import AbstractWString
from .rosidl_parser.definition import ACTION_FEEDBACK_SUFFIX
from .rosidl_parser.definition import ACTION_GOAL_SUFFIX
from .rosidl_parser.definition import ACTION_RESULT_SUFFIX
from .rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from .rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from .rosidl_parser.definition import BasicType
from .rosidl_parser.definition import BOOLEAN_TYPE
from .rosidl_parser.definition import CHARACTER_TYPES
from .rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
from .rosidl_parser.definition import INTEGER_TYPES
from .rosidl_parser.definition import NamespacedType
from .rosidl_parser.definition import OCTET_TYPE
from .rosidl_parser.definition import UNSIGNED_INTEGER_TYPES

message_typename = '::'.join(message.structure.namespaced_type.namespaced_name())

# Common Windows macros that may interfere with user defined constants
msvc_common_macros = ('DELETE', 'ERROR', 'NO_ERROR')
}@
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Collect necessary include directives for all members
@{
from collections import OrderedDict
from .rosidl_pycommon.api import convert_camel_case_to_lower_case_underscore
includes = OrderedDict()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, NamespacedType):
        if (
            message.structure.namespaced_type.namespaces[-1] in ['action', 'srv'] and (
            type_.name.endswith(SERVICE_REQUEST_MESSAGE_SUFFIX) or
            type_.name.endswith(SERVICE_RESPONSE_MESSAGE_SUFFIX))
        ):
            typename = type_.name.rsplit('_', 1)[0]
            if typename == message.structure.namespaced_type.name.rsplit('_', 1)[0]:
                continue
        if (
            type_.name.endswith(ACTION_GOAL_SUFFIX) or
            type_.name.endswith(ACTION_RESULT_SUFFIX) or
            type_.name.endswith(ACTION_FEEDBACK_SUFFIX)
        ):
            typename = type_.name.rsplit('_', 1)[0]
        else:
            typename = type_.name
        member_names = includes.setdefault(
            '/'.join((type_.namespaces + ['detail', convert_camel_case_to_lower_case_underscore(typename)])) + '__struct.hpp', [])
        member_names.append(member.name)
}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@[if includes]@

// Include directives for member types
@[    for header_file, member_names in includes.items()]@
@[        for member_name in member_names]@
// Member '@(member_name)'
@[        end for]@
@[        if header_file in include_directives]@
// already included above
// @
@[        else]@
@{include_directives.add(header_file)}@
@[        end if]@
#include "@(header_file)"
@[    end for]@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@{
deprecated_macro_name = \
    '__'.join(['DEPRECATED', package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])
}@
#ifndef _WIN32
# define @(deprecated_macro_name) __attribute__((deprecated))
#else
# define @(deprecated_macro_name) __declspec(deprecated)
#endif

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{

@[end for]@
// message struct
template<class ContainerAllocator>
struct @(message.structure.namespaced_type.name)_
{
  using Type = @(message.structure.namespaced_type.name)_<ContainerAllocator>;

@{
# The creation of the constructors for messages is a bit complicated.  The goal
# is to have a constructor where the user can control how the fields of the
# message get initialized via the _init parameter to the constructor.  See
# http://design.ros2.org/articles/generated_interfaces_cpp.html#constructors
# for a detailed explanation of the different _init parameters.
init_list, alloc_list, member_list = create_init_alloc_and_member_lists(message)

def generate_default_string(membset):
    from .rosidl_generator_cpp.api import msg_type_only_to_cpp
    from .rosidl_generator_cpp.api import msg_type_to_cpp
    strlist = []
    for member in membset.members:
        if member.default_value is not None:
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if isinstance(member.default_value, list):
                if all(v == member.default_value[0] for v in member.default_value):
                    # Specifying type for std::fill because of MSVC 14.12 warning about casting 'const int' to smaller types (C4244)
                    # For more info, see https://github.com/ros2/rosidl/issues/309
                    # TODO(jacobperron): Investigate reason for build warnings on Windows
                    # TODO(jacobperron): Write test case for this path of execution
                    strlist.append('std::fill<typename %s::iterator, %s>(this->%s.begin(), this->%s.end(), %s);' % (msg_type_to_cpp(member.type), msg_type_only_to_cpp(member.type), member.name, member.name, member.default_value[0]))
                else:
                    for index, val in enumerate(member.default_value):
                        strlist.append('this->%s[%d] = %s;' % (member.name, index, val))
            else:
                strlist.append('this->%s = %s;' % (member.name, member.default_value))

    return strlist

def generate_zero_string(membset, fill_args):
    from .rosidl_generator_cpp.api import msg_type_only_to_cpp
    from .rosidl_generator_cpp.api import msg_type_to_cpp
    strlist = []
    for member in membset.members:
        if isinstance(member.zero_value, list):
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if member.zero_need_array_override:
                strlist.append('this->%s.fill(%s{%s});' % (member.name, msg_type_only_to_cpp(member.type), fill_args))
            else:
                # Specifying type for std::fill because of MSVC 14.12 warning about casting 'const int' to smaller types (C4244)
                # For more info, see https://github.com/ros2/rosidl/issues/309
                # TODO(jacobperron): Investigate reason for build warnings on Windows
                strlist.append('std::fill<typename %s::iterator, %s>(this->%s.begin(), this->%s.end(), %s);' % (msg_type_to_cpp(member.type), msg_type_only_to_cpp(member.type), member.name, member.name, member.zero_value[0]))
        else:
            strlist.append('this->%s = %s;' % (member.name, member.zero_value))
    return strlist
}@
  explicit @(message.structure.namespaced_type.name)_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
@[if init_list]@
  : @(',\n    '.join(init_list))
@[end if]@
  {
@[if not member_list]@
    (void)_init;
@[end if]@
@{
default_value_members = [m for m in member_list if m.members[0].default_value]
zero_value_members = [m for m in member_list if m.members[0].zero_value]
non_defaulted_zero_initialized_members = [
    m for m in member_list
    if (m.members[0].zero_value or m.members[0].zero_need_array_override) and not m.members[0].default_value
]
}@
@[if default_value_members]@
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
@[  for membset in default_value_members]@
@[    for line in generate_default_string(membset)]@
      @(line)
@[    end for]@
@[  end for]@
@[  if zero_value_members]@
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
@[    for membset in zero_value_members]@
@[      for line in generate_zero_string(membset, '_init')]@
      @(line)
@[      end for]@
@[    end for]@
@[  end if]@
    }
@[  end if]@
@[  if non_defaulted_zero_initialized_members]@
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
@[  for membset in non_defaulted_zero_initialized_members]@
@[    for line in generate_zero_string(membset, '_init')]@
@[      if line]@
      @(line)
@[      end if]@
@[    end for]@
@[  end for]@
    }
@[end if]@
  }

  explicit @(message.structure.namespaced_type.name)_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
@[if alloc_list]@
  : @(',\n    '.join(alloc_list))
@[end if]@
  {
@[if not member_list]@
    (void)_init;
@[end if]@
@[if not alloc_list]@
    (void)_alloc;
@[end if]@
@[if default_value_members]@
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
@[  for membset in default_value_members]@
@[    for line in generate_default_string(membset)]@
      @(line)
@[    end for]@
@[  end for]@
@[  if zero_value_members]@
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
@[    for membset in zero_value_members]@
@[      for line in generate_zero_string(membset, '_alloc, _init')]@
      @(line)
@[      end for]@
@[    end for]@
@[  end if]@
    }
@[end if]@
@[if non_defaulted_zero_initialized_members]@
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
@[  for membset in non_defaulted_zero_initialized_members]@
@[    for line in generate_zero_string(membset, '_alloc, _init')]@
      @(line)
@[    end for]@
@[  end for]@
    }
@[end if]@
  }

  // field types and members
@[for member in message.structure.members]@
  using _@(member.name)_type =
    @(msg_type_to_cpp(member.type));
  _@(member.name)_type @(member.name);
@[end for]@

@[if len(message.structure.members) != 1 or message.structure.members[0].name != EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
  // setters for named parameter idiom
@[  for member in message.structure.members]@
  Type & set__@(member.name)(
    const @(msg_type_to_cpp(member.type)) & _arg)
  {
    this->@(member.name) = _arg;
    return *this;
  }
@[  end for]@
@[end if]@

  // constant declarations
@[for constant in message.constants]@
@[ if constant.name in msvc_common_macros]@
  // guard against '@(constant.name)' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(@(constant.name))
#    pragma push_macro("@(constant.name)")
#    undef @(constant.name)
#  endif
#endif
@[ end if]@
@[ if isinstance(constant.type, AbstractString)]@
  static const @(MSG_TYPE_TO_CPP['string']) @(constant.name);
@[ elif isinstance(constant.type, AbstractWString)]@
  static const @(MSG_TYPE_TO_CPP['wstring']) @(constant.name);
@[ else]@
  static constexpr @(MSG_TYPE_TO_CPP[constant.type.typename]) @(constant.name) =
@[  if isinstance(constant.type, BasicType)]@
@[   if constant.type.typename in (*INTEGER_TYPES, *CHARACTER_TYPES, BOOLEAN_TYPE, OCTET_TYPE)]@
    @(int(constant.value))@
@[    if constant.type.typename in UNSIGNED_INTEGER_TYPES]@
u@
@[    end if]@
@[   elif constant.type.typename == 'float']@
    @(constant.value)f@
@[   else]@
    @(constant.value)@
@[   end if];
@[  else]@
    @(constant.value);
@[  end if]@
@[ end if]@
@[ if constant.name in msvc_common_macros]@
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("@(constant.name)")
#endif
@[ end if]@
@[end for]@

  // pointer types
  using RawPtr =
    @(message_typename)_<ContainerAllocator> *;
  using ConstRawPtr =
    const @(message_typename)_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<@(message_typename)_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<@(message_typename)_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      @(message_typename)_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<@(message_typename)_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      @(message_typename)_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<@(message_typename)_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<@(message_typename)_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<@(message_typename)_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(message_typename)_<ContainerAllocator>>
    Ptr;
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(message_typename)_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const @(message.structure.namespaced_type.name)_ & other) const
  {
@[if not message.structure.members]@
    (void)other;
@[end if]@
@[for member in message.structure.members]@
    if (this->@(member.name) != other.@(member.name)) {
      return false;
    }
@[end for]@
    return true;
  }
  bool operator!=(const @(message.structure.namespaced_type.name)_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct @(message.structure.namespaced_type.name)_

// alias to use template instance with default allocator
using @(message.structure.namespaced_type.name) =
  @(message_typename)_<std::allocator<void>>;

// constant definitions
@[for c in message.constants]@
@[ if c.name in msvc_common_macros]@
// guard against '@(c.name)' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(@(c.name))
#    pragma push_macro("@(c.name)")
#    undef @(c.name)
#  endif
#endif
@[ end if]@
@[ if isinstance(c.type, AbstractString)]@
template<typename ContainerAllocator>
const @(MSG_TYPE_TO_CPP['string'])
@(message.structure.namespaced_type.name)_<ContainerAllocator>::@(c.name) = "@(escape_string(c.value))";
@[ elif isinstance(c.type, AbstractWString)]@
template<typename ContainerAllocator>
const @(MSG_TYPE_TO_CPP['wstring'])
@(message.structure.namespaced_type.name)_<ContainerAllocator>::@(c.name) = u"@(escape_wstring(c.value))";
@[ else ]@
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr @(MSG_TYPE_TO_CPP[c.type.typename]) @(message.structure.namespaced_type.name)_<ContainerAllocator>::@(c.name);
#endif  // __cplusplus < 201703L
@[ end if]@
@[ if c.name in msvc_common_macros]@
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("@(c.name)")
#endif
@[ end if]@
@[end for]@
@
@[for ns in reversed(message.structure.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
