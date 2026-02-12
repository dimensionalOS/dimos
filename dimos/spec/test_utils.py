# Copyright 2025-2026 Dimensional Inc.
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

from typing import Protocol

import pytest

from dimos.spec.utils import Spec, is_spec, spec_annotation_compliance, spec_structural_compliance


class NormalProtocol(Protocol):
    def foo(self) -> int: ...


class SpecProtocol(Spec, Protocol):
    def foo(self) -> int: ...


def test_is_spec_recognizes_spec_protocol() -> None:
    assert is_spec(SpecProtocol) is True


def test_is_spec_rejects_plain_protocol_and_base() -> None:
    assert is_spec(NormalProtocol) is False
    assert is_spec(Spec) is False


def test_is_spec_rejects_non_type() -> None:
    assert is_spec(object()) is False


class MySpec(Spec, Protocol):
    def foo(self) -> int:
        return 1


class StructurallyCompliant:
    def foo(self) -> str:
        return "ok"


class FullyCompliant:
    def foo(self) -> int:
        return 1


class NotCompliant:
    pass


def test_spec_structural_compliance_matches_by_structure() -> None:
    assert spec_structural_compliance(NotCompliant(), MySpec) is False
    assert spec_structural_compliance(StructurallyCompliant(), MySpec) is True
    assert spec_structural_compliance(FullyCompliant(), MySpec) is True


def test_spec_structural_compliance_rejects_non_spec() -> None:
    with pytest.raises(TypeError):
        spec_structural_compliance(StructurallyCompliant(), NormalProtocol)  # type: ignore[arg-type]


def test_spec_annotation_compliance_requires_matching_annotations() -> None:
    assert spec_annotation_compliance(StructurallyCompliant(), MySpec) is False
    assert spec_annotation_compliance(FullyCompliant(), MySpec) is True


def test_spec_annotation_compliance_rejects_non_spec() -> None:
    with pytest.raises(TypeError):
        spec_annotation_compliance(StructurallyCompliant(), NormalProtocol)  # type: ignore[arg-type]
