# Copyright 2026 Dimensional Inc.
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

import inspect
from typing import Any, Protocol, runtime_checkable

from annotation_protocol import AnnotationProtocol
from typing_extensions import is_protocol


# Allows us to differentiate plain Protocols from Module-Spec Protocols
class Spec(Protocol):
    pass


def is_spec(cls: Any) -> bool:
    """
    Example:
        class NormalProtocol(Protocol):
            def foo(self) -> int: ...

        class SpecProtocol(Spec, Protocol):
            def foo(self) -> int: ...

        is_spec(NormalProtocol)  # False
        is_spec(SpecProtocol)    # True
    """
    return inspect.isclass(cls) and is_protocol(cls) and Spec in cls.__mro__ and cls is not Spec


def spec_structural_compliance(
    obj: Any,
    spec: Any,
) -> bool:
    """
    Example:
        class MySpec(Spec, Protocol):
            def foo(self) -> int: ...

        class StructurallyCompliant1:
            def foo(self) -> list[list[list[list[list[int]]]]]: ...
        class StructurallyCompliant2:
            def foo(self) -> str: ...
        class FullyCompliant:
            def foo(self) -> int: ...
        class NotCompliant:
            ...

        assert False == spec_structural_compliance(NotCompliant(), MySpec)
        assert True == spec_structural_compliance(StructurallyCompliant1(), MySpec)
        assert True == spec_structural_compliance(StructurallyCompliant2(), MySpec)
        assert True == spec_structural_compliance(FullyCompliant(), MySpec)
    """
    if not is_spec(spec):
        raise TypeError("Trying to check if `obj` implements `spec` but spec itself was not a Spec")

    # python's built-in protocol check ignores annotations (only structural check)
    return isinstance(obj, runtime_checkable(spec))


def spec_annotation_compliance(
    obj: Any,
    proto: Any,
) -> bool:
    """
    Example:
        class MySpec(Spec, Protocol):
            def foo(self) -> int: ...

        class StructurallyCompliant1:
            def foo(self) -> list[list[list[list[list[int]]]]]: ...
        class FullyCompliant:
            def foo(self) -> int: ...

        assert False == spec_annotation_compliance(StructurallyCompliant1(), MySpec)
        assert True == spec_structural_compliance(FullyCompliant(), MySpec)
    """
    if not is_spec(proto):
        raise TypeError("Not a Spec")

    # Build a *strict* runtime protocol dynamically
    strict_proto = type(
        f"Strict{proto.__name__}",
        (AnnotationProtocol,),
        dict(proto.__dict__),
    )

    return isinstance(obj, strict_proto)


def get_protocol_method_signatures(proto: type[object]) -> dict[str, inspect.Signature]:
    """
    Return a mapping of method_name -> inspect.Signature
    for all methods required by a Protocol.
    """
    if not is_protocol(proto):
        raise TypeError(f"{proto} is not a Protocol")

    methods: dict[str, inspect.Signature] = {}

    # Walk MRO so inherited protocol methods are included
    for cls in reversed(proto.__mro__):
        if cls is Protocol:  # type: ignore[comparison-overlap]
            continue

        for name, value in cls.__dict__.items():
            if name.startswith("_"):
                continue

            if callable(value):
                try:
                    sig = inspect.signature(value)
                except (TypeError, ValueError):
                    continue

                methods[name] = sig

    return methods
