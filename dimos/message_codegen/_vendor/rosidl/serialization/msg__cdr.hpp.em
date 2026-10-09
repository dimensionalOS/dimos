@{
from .rosidl_parser.definition import AbstractNestedType, AbstractWString, Array, BasicType, BoundedSequence, NamespacedType
}@
@{

# Generates the definition for the serialization family of methods given a structure member
#   member: the member to serialize
#   suffix: the suffix name of the method. Will be used in case of recursion

def generate_member_for_cdr_serialize(member, suffix):
  from .rosidl_generator_cpp import msg_type_only_to_cpp
  from .rosidl_generator_cpp import msg_type_to_cpp
  from .rosidl_parser.definition import AbstractGenericString
  from .rosidl_parser.definition import AbstractNestedType
  from .rosidl_parser.definition import AbstractSequence
  from .rosidl_parser.definition import AbstractWString
  from .rosidl_parser.definition import Array
  from .rosidl_parser.definition import BasicType
  from .rosidl_parser.definition import BoundedSequence
  from .rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Member: %s' % (member.name))
  if isinstance(member.type, AbstractNestedType):
    strlist.append('{')
    if isinstance(member.type, Array):
      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)):
        strlist.append('  cdr << ros_message.%s;' % (member.name))
      else:
        strlist.append('  for (size_t i = 0; i < %d; i++) {' % (member.type.size))
        if isinstance(member.type.value_type, NamespacedType):
          strlist.append('    %s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
          strlist.append('      ros_message.%s[i],' % (member.name))
          strlist.append('      cdr);')
        else:
          strlist.append('    rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s[i]);' % (member.name))
        strlist.append('  }')
    else:
      if isinstance(member.type, BoundedSequence) or isinstance(member.type.value_type, (NamespacedType, AbstractWString)):
        strlist.append('  size_t size = ros_message.%s.size();' % (member.name))
        if isinstance(member.type, BoundedSequence):
          strlist.append('  if (size > %d) {' % (member.type.maximum_size))
          strlist.append('    throw std::runtime_error("array size exceeds upper bound");')
          strlist.append('  }')
      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)) and not isinstance(member.type, BoundedSequence):
        strlist.append('  cdr << ros_message.%s;' % (member.name))
      else:
        strlist.append('  cdr << static_cast<uint32_t>(size);')
        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename not in ('boolean', 'wchar'):
          strlist.append('  if (size > 0) {')
          strlist.append('    cdr.serialize_array(&(ros_message.%s[0]), size);' % (member.name))
          strlist.append('  }')
        else:
          strlist.append('  for (size_t i = 0; i < size; i++) {')
          if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean':
            strlist.append('    cdr << (ros_message.%s[i] ? true : false);' % (member.name))
          elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar':
            strlist.append('    cdr << static_cast<wchar_t>(ros_message.%s[i]);' % (member.name))
          elif isinstance(member.type.value_type, AbstractWString):
            strlist.append('    rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s[i]);' % (member.name))
          elif not isinstance(member.type.value_type, NamespacedType):
            strlist.append('    cdr << ros_message.%s[i];' % (member.name))
          else:
            strlist.append('    %s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
            strlist.append('      ros_message.%s[i],' % (member.name))
            strlist.append('      cdr);')
          strlist.append('  }')
    strlist.append('}')
  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean':
    strlist.append('cdr << (ros_message.%s ? true : false);' % (member.name))
  elif isinstance(member.type, BasicType) and member.type.typename == 'wchar':
    strlist.append('cdr << static_cast<wchar_t>(ros_message.%s);' % (member.name))
  elif isinstance(member.type, AbstractWString):
    strlist.append('{')
    strlist.append('  rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s);' % (member.name))
    strlist.append('}')
  elif not isinstance(member.type, NamespacedType):
    strlist.append('cdr << ros_message.%s;' % (member.name))
  else:
    strlist.append('%s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.namespaces)), suffix))
    strlist.append('  ros_message.%s,' % (member.name))
    strlist.append('  cdr);')
  return strlist
}@

bool
inline
cdr_serialize(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
@[for member in message.structure.members]@
@[  for line in generate_member_for_cdr_serialize(member, '')]@
  @(line)
@[  end for]@

@[end for]@
  return true;
}

bool
inline
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
  {
@[    if isinstance(member.type, Array)]@
@[      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString))]@
    cdr >> ros_message.@(member.name);
@[      else]@
    for (size_t i = 0; i < @(member.type.size); i++) {
@[        if isinstance(member.type.value_type, NamespacedType)]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.@(member.name)[i]);
@[        else]@
      bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name)[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to deserialize u16string\n");
        return false;
      }
@[        end if]@
    }
@[      end if]@
@[    else]@
@[      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)) and not isinstance(member.type, BoundedSequence)]@
    cdr >> ros_message.@(member.name);
@[      else]@
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    ros_message.@(member.name).resize(size);
@[        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename not in ('boolean', 'wchar')]@
    if (size > 0) {
      cdr.deserialize_array(&(ros_message.@(member.name)[0]), size);
    }
@[        else]@
    for (size_t i = 0; i < size; i++) {
@[            if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean']@
      uint8_t tmp;
      cdr >> tmp;
      if (tmp > 1) { return false; }
      ros_message.@(member.name)[i] = tmp ? true : false;
@[            elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar']@
      wchar_t tmp;
      cdr >> tmp;
      ros_message.@(member.name)[i] = static_cast<char16_t>(tmp);
@[            elif isinstance(member.type.value_type, AbstractWString)]@
      bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name)[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to deserialize u16string\n");
        return false;
      }
@[            elif not isinstance(member.type.value_type, NamespacedType)]@
      cdr >> ros_message.@(member.name)[i];
@[            else]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.@(member.name)[i]);
@[            end if]@
    }
@[          end if]@
@[      end if]@
@[    end if]@
  }
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@
  {
    uint8_t tmp;
    cdr >> tmp;
    if (tmp > 1) { return false; }
    ros_message.@(member.name) = tmp ? true : false;
  }
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'wchar']@
  {
    wchar_t tmp;
    cdr >> tmp;
    ros_message.@(member.name) = static_cast<char16_t>(tmp);
  }
@[  elif isinstance(member.type, AbstractWString)]@
  {
    bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name));
    if (!succeeded) {
      fprintf(stderr, "failed to deserialize u16string\n");
      return false;
    }
  }
@[  elif not isinstance(member.type, NamespacedType)]@
  cdr >> ros_message.@(member.name);
@[  else]@
  @('::'.join(member.type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.@(member.name));
@[  end if]@

@[end for]@
  return true;
}  // NOLINT(readability/fn_size)

@{

# Generates the definition for the get_serialized_size family of methods given a structure member
#   member: the member to serialize
#   suffix: the suffix name of the method. Will be used in case of recursion

def generate_member_for_get_serialized_size(member, suffix):
  from .rosidl_generator_cpp import msg_type_only_to_cpp
  from .rosidl_generator_cpp import msg_type_to_cpp
  from .rosidl_parser.definition import AbstractGenericString
  from .rosidl_parser.definition import AbstractNestedType
  from .rosidl_parser.definition import AbstractSequence
  from .rosidl_parser.definition import AbstractWString
  from .rosidl_parser.definition import Array
  from .rosidl_parser.definition import BasicType
  from .rosidl_parser.definition import BoundedSequence
  from .rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Member: %s' % (member.name))

  if isinstance(member.type, AbstractNestedType):
    strlist.append('{')
    if isinstance(member.type, Array):
      strlist.append('  size_t array_size = %d;' % (member.type.size))
    else:
      strlist.append('  size_t array_size = ros_message.%s.size();' % (member.name))
      if isinstance(member.type, BoundedSequence):
        strlist.append('  if (array_size > %d) {' % (member.type.maximum_size))
        strlist.append('    throw std::runtime_error("array size exceeds upper bound");')
        strlist.append('  }')
      strlist.append('  current_alignment += padding +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, padding);')
    if isinstance(member.type.value_type, AbstractGenericString):
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment += padding +')
      strlist.append('      eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type.value_type, AbstractWString):
        strlist.append('      wchar_size *')
      strlist.append('      (ros_message.%s[index].size() + 1);' % (member.name))
      strlist.append('  }')
    elif isinstance(member.type.value_type, BasicType):
      strlist.append('  size_t item_size = sizeof(ros_message.%s[0]);' % (member.name))
      strlist.append('  current_alignment += array_size * item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
    else:
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment +=')
      strlist.append('      %s::typesupport_fastrtps_cpp::get_serialized_size%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
      strlist.append('      ros_message.%s[index], current_alignment);' % (member.name))
      strlist.append('  }')
    strlist.append('}')
  else:
    if isinstance(member.type, AbstractGenericString):
      strlist.append('current_alignment += padding +')
      strlist.append('  eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type, AbstractWString):
        strlist.append('  wchar_size *')
      strlist.append('  (ros_message.%s.size() + 1);' % (member.name))
    elif isinstance(member.type, BasicType):
      strlist.append('{')
      strlist.append('  size_t item_size = sizeof(ros_message.%s);' % (member.name))
      strlist.append('  current_alignment += item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
      strlist.append('}')
    else:
      strlist.append('current_alignment +=')
      strlist.append('  %s::typesupport_fastrtps_cpp::get_serialized_size%s(' % (('::'.join(member.type.namespaces)), suffix))
      strlist.append('  ros_message.%s, current_alignment);' % (member.name))

  return strlist;
}@

size_t
inline
get_serialized_size(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

@[for member in message.structure.members]@
@[  for line in generate_member_for_get_serialized_size(member, '')]@
  @(line)
@[  end for]@

@[end for]@
  return current_alignment - initial_alignment;
}

