@{
from .rosidl_parser.definition import AbstractNestedType, AbstractWString, Array, BasicType, BoundedSequence, Message, NamespacedType
}@
@[for message in content.get_elements_of_type(Message)]@
namespace @('::'.join(message.structure.namespaced_type.namespaces))::typesupport_fastrtps_cpp {
@{TEMPLATE('msg__cdr.hpp.em', message=message, package_name=package_name, interface_path=interface_path)}@
}
@[end for]@
