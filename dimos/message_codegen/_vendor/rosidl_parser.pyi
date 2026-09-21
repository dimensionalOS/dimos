from typing import Any

class InvalidSpecification(Exception): ...
class InvalidValue(Exception): ...

class Type:
    pkg_name: str | None
    type: str
    string_upper_bound: int | None
    is_array: bool
    array_size: int | None
    is_upper_bound: bool

class Field:
    type: Type
    name: str
    default_value: Any

class Constant:
    type: str
    name: str
    value: Any

class MessageSpecification:
    fields: list[Field]
    constants: list[Constant]

def parse_message_string(
    pkg_name: str, msg_name: str, message_string: str
) -> MessageSpecification: ...
