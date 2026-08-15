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

import pytest

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.python_native_bootstrap import load_class, validate_runtime
from dimos.core.python_native_module import PythonNativeModule, PythonNativeModuleConfig


class Contract(PythonNativeModule):
    implementation = "unused:Runtime"
    config: PythonNativeModuleConfig

    @rpc
    def value(self, amount: int) -> int:
        raise NotImplementedError

    @skill
    def change(self, amount: int) -> str:
        raise NotImplementedError


class Runtime(Contract):
    @rpc
    def value(self, amount: int) -> int:
        return amount

    @skill
    def change(self, amount: int) -> str:
        return str(amount)


def test_runtime_must_override_every_contract_rpc() -> None:
    class Missing(Contract):
        @skill
        def change(self, amount: int) -> str:
            return str(amount)

    with pytest.raises(TypeError, match="must override contract RPC 'value'"):
        validate_runtime(Contract, Missing)


def test_runtime_must_preserve_rpc_signature() -> None:
    class Wrong(Runtime):
        @rpc
        def value(self, amount: str) -> int:
            return len(amount)

    with pytest.raises(TypeError, match="incompatible signature"):
        validate_runtime(Contract, Wrong)


def test_runtime_must_preserve_skill_classification() -> None:
    class Wrong(Runtime):
        @rpc
        def change(self, amount: int) -> str:
            return str(amount)

    with pytest.raises(TypeError, match="preserve skill classification"):
        validate_runtime(Contract, Wrong)


def test_valid_runtime_subclass_is_accepted() -> None:
    validate_runtime(Contract, Runtime)


@pytest.mark.parametrize("reference", ["", "Runtime", ":Runtime", "module:"])
def test_load_class_rejects_invalid_reference(reference: str) -> None:
    with pytest.raises(ValueError, match="Invalid import reference"):
        load_class(reference)
