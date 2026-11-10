from collections.abc import Mapping
from typing import ClassVar

from pydantic import JsonValue

class Type:
    BOOL: ClassVar[Type]
    DOUBLE: ClassVar[Type]
    DYN: ClassVar[Type]
    ERROR: ClassVar[Type]
    INT: ClassVar[Type]
    STRING: ClassVar[Type]
    UNKNOWN: ClassVar[Type]
    @staticmethod
    def List(element_type: Type) -> Type: ...
    @staticmethod
    def Map(key_type: Type, element_type: Type) -> Type: ...
    def name(self) -> str: ...

class Value:
    def plain_value(self) -> JsonValue: ...
    def type(self) -> Type: ...

class Expression:
    def eval(self, data: Mapping[str, JsonValue] | None = ...) -> Value: ...
    def return_type(self) -> Type: ...

class Env:
    def compile(self, expression: str, disable_check: bool = ...) -> Expression: ...

def NewEnv(variables: Mapping[str, Type] | None = ...) -> Env: ...
