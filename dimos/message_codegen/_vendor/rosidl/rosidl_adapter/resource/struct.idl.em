@#typedefs for arrays need to be defined outside of the struct
@{
from collections import OrderedDict

from .rosidl_adapter.msg import get_idl_type
from .rosidl_adapter.msg import to_idl_literal
from .rosidl_adapter.msg import string_to_idl_string_literal

typedefs = OrderedDict()
def get_idl_type_identifier(idl_type):
    return idl_type.replace('::', '__') \
        .replace('<', '__').replace('>', '') \
        .replace('[', '__').replace(']', '')
}@
@[for field in msg.fields]@
@{
idl_type = get_idl_type(field.type)
}@
@[  if field.type.is_fixed_size_array()]@
@{
idl_base_type = idl_type.split('[', 1)[0]
idl_base_type_identifier = idl_base_type.replace('::', '__')
# only necessary for complex types
if idl_base_type_identifier != idl_base_type:
    if idl_base_type_identifier not in typedefs:
        typedefs[idl_base_type_identifier] = idl_base_type
    else:
        assert typedefs[idl_base_type_identifier] == idl_base_type
idl_type_identifier = get_idl_type_identifier(idl_type) + '[' + str(field.type.array_size) + ']'
if idl_type_identifier not in typedefs:
    typedefs[idl_type_identifier] = idl_base_type_identifier
else:
    assert typedefs[idl_type_identifier] == idl_base_type_identifier
}@
@[  end if]@
@[end for]@
@[for k, v in typedefs.items()]@
    typedef @(v) @(k);
@[end for]@
@[if msg.constants]@
    module @(msg.msg_name)_Constants {
@[  for constant in msg.constants]@
@[    if constant.annotations.get('comment', [])]@
      @@verbatim (language="comment", text=
@[      for i, line in enumerate(constant.annotations['comment'])]@
        @(string_to_idl_string_literal(line))@
@[        if i < len(constant.annotations.get('comment')) - 1]@
 "\n"@
@[        end if]@
@[      end for]@
)
@[    end if]@
      const @(get_idl_type(constant.type)) @(constant.name) = @(to_idl_literal(get_idl_type(constant.type), constant.value));
@[  end for]@
    };
@[end if]@
@#
@[if msg.annotations.get('comment', [])]@
    @@verbatim (language="comment", text=@
@[  for i, line in enumerate(msg.annotations['comment'])]
      @(string_to_idl_string_literal(line))@
@[    if i < len(msg.annotations.get('comment')) - 1]@
 "\n"@
@[    end if]@
@[  end for]@
)
@[end if]@
    struct @(msg.msg_name) {
@[if msg.fields]@
@[  for i, field in enumerate(msg.fields)]@
@[    if i > 0]@

@[    end if]@
@[    if field.annotations.get('comment', [])]@
      @@verbatim (language="comment", text=@
@[      for j, line in enumerate(field.annotations['comment'])]
        @(string_to_idl_string_literal(line))@
@[        if j < len(field.annotations.get('comment')) - 1]@
 "\n"@
@[        end if]@
@[      end for]@
)
@[    end if]@
@[    if field.default_value is not None]@
      @@default (value=@(to_idl_literal(get_idl_type(field.type), field.default_value)))
@[    end if]@
@[    if 'unit' in field.annotations]@
      @@unit (value=@(string_to_idl_string_literal(field.annotations['unit'])))
@[    end if]@
@{
idl_type = get_idl_type(field.type)
}@
@[    if field.type.is_fixed_size_array()]@
@{
idl_type = get_idl_type_identifier(idl_type)
}@
@[    end if]@
      @(idl_type) @(field.name);
@[  end for]@
@[else]@
      uint8 structure_needs_at_least_one_member;
@[end if]@
    };
