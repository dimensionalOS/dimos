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

from typing import Protocol

from dimos.core import Module, rpc
from dimos.core.blueprints import autoconnect
from dimos.spec.utils import Spec


# this would be defined in some other file (this could be imported from a library)
class Calculator(Module):
    @rpc
    def compute1(self, a: int, b: int) -> int:
        return a + b

    @rpc
    def compute2(self, a: float, b: float) -> float:
        return a + b


# what your module needs/expects
class ComputeSpec(Spec, Protocol):
    @rpc
    def compute1(self, a: int, b: int) -> int: ...

    @rpc
    def compute2(self, a: float, b: float) -> float: ...


class Client(Module):
    # this says: "hey dimos, give me access to a module that has a compute1 and compute2 method"
    calc: ComputeSpec

    @rpc
    def start(self) -> None:
        print("compute1:", self.calc.compute1(2, 3))
        print("compute2:", self.calc.compute2(1.5, 2.5))


if __name__ == "__main__":
    autoconnect(
        Calculator.blueprint(),
        Client.blueprint(),
    ).build().loop()
