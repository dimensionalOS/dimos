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
import sys
from types import UnionType
from typing import Any, Protocol, Union, get_args, get_origin, runtime_checkable

if sys.version_info >= (3, 13):
    from typing import get_protocol_members, is_protocol
else:
    from typing_extensions import get_protocol_members, is_protocol


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

    # Structural compliance (every member present) is a prerequisite.
    if not isinstance(obj, runtime_checkable(proto)):
        return False

    # On top of structure, every method the spec declares must match by signature
    # (return + argument annotations). Data attributes are not signature-able, so
    # only their presence -- already verified above -- is required.
    obj_cls = obj if isinstance(obj, type) else type(obj)
    for name in get_protocol_members(proto):
        try:
            spec_sig = inspect.signature(getattr(proto, name), eval_str=True)
        except (AttributeError, TypeError, ValueError):
            continue  # data attribute, not a method -- only its presence matters
        try:
            impl_sig = inspect.signature(getattr(obj_cls, name), eval_str=True)
        except (AttributeError, TypeError, ValueError):
            return False  # spec declares a method here but the impl has no callable
        if not _signatures_compatible(spec_sig, impl_sig):
            return False
    return True


def _annotation_compatible(spec_ann: Any, impl_ann: Any) -> bool:
    """Return True if an implementation's ``impl_ann`` satisfies the spec's ``spec_ann``.

    Missing (``empty``), ``Any`` and ``None`` spec annotations accept any implementation
    type, and an ``Any`` implementation annotation satisfies any spec. A union spec
    annotation is satisfied by any subset of its members.
    """
    if spec_ann is inspect.Parameter.empty or spec_ann is Any or spec_ann is None:
        return True
    if impl_ann is Any:
        return True

    spec_origin = get_origin(spec_ann)
    if spec_origin is Union or spec_origin is UnionType:
        spec_types = set(get_args(spec_ann))
        impl_origin = get_origin(impl_ann)
        if impl_origin is Union or impl_origin is UnionType:
            impl_types = set(get_args(impl_ann))
        else:
            impl_types = {impl_ann}
        return spec_types >= impl_types

    return bool(spec_ann == impl_ann)


def _signatures_compatible(spec_sig: inspect.Signature, impl_sig: inspect.Signature) -> bool:
    """Return True if ``impl_sig`` satisfies ``spec_sig`` by return and argument annotations."""
    if not _annotation_compatible(spec_sig.return_annotation, impl_sig.return_annotation):
        return False

    # Re-shape the implementation parameters into positional/keyword arguments and bind
    # them against the spec signature, which validates arity and parameter names.
    has_var_args = any(
        p.kind is inspect.Parameter.VAR_POSITIONAL for p in spec_sig.parameters.values()
    )
    args: list[inspect.Parameter] = []
    kwargs: dict[str, inspect.Parameter] = {}
    for p in impl_sig.parameters.values():
        # An extra *optional* impl parameter (not declared by the spec, has a default)
        # does not break substitutability -- a spec-driven caller never passes it -- so
        # ignore it. Extra *required* params are kept, so binding fails and the impl is
        # rejected.
        if p.default is not inspect.Parameter.empty and p.name not in spec_sig.parameters:
            continue
        if p.kind is inspect.Parameter.POSITIONAL_ONLY:
            args.append(p)
        elif p.kind is inspect.Parameter.KEYWORD_ONLY:
            kwargs[p.name] = p
        elif p.kind is inspect.Parameter.POSITIONAL_OR_KEYWORD:
            if has_var_args:
                args.append(p)
            else:
                kwargs[p.name] = p

    try:
        bound = spec_sig.bind(*args, **kwargs)
    except TypeError:
        return False

    for spec_param in spec_sig.parameters.values():
        if spec_param.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
            continue
        if spec_param.name not in bound.arguments:
            return False  # spec declares this parameter but the impl does not provide it
        impl_param = bound.arguments[spec_param.name]
        if (
            spec_param.kind is not inspect.Parameter.POSITIONAL_ONLY
            and spec_param.name != impl_param.name
        ):
            return False
        if (
            spec_param.kind is inspect.Parameter.POSITIONAL_OR_KEYWORD
            and impl_param.kind is inspect.Parameter.POSITIONAL_ONLY
        ):
            return False
        if not _annotation_compatible(spec_param.annotation, impl_param.annotation):
            return False
    return True


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
