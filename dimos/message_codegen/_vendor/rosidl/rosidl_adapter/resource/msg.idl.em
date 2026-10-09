// generated from rosidl_adapter/resource/msg.idl.em
// with input from @(pkg_name)/@(relative_input_file)
// generated code does not contain a copyright notice

@{
from .rosidl_adapter.msg import get_include_file
include_files = set()
for field in msg.fields:
    include_file = get_include_file(field.type)
    if include_file is not None:
        include_files.add(include_file)
}@
@[for include_file in sorted(include_files)]@
#include "@(include_file)"
@[end for]@

module @(pkg_name) {
  module msg {
@{
TEMPLATE(
    'struct.idl.em',
    msg=msg,
)
}@
  };
};
