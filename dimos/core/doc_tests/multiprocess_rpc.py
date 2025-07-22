# Copyright 2025 Dimensional Inc.
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

from typing import Callable

from dimos.core import Module, rpc, start


class CtrlModule(Module):
    remote_add: Callable[[int], int] = None

    def __init__(self, remote_add=None):
        self.remote_add = remote_add
        super().__init__()

    @rpc
    def call(self, n):
        return self.remote_add(n)


class AddModule(Module):
    @rpc
    def add(self, n):
        return n + n


if __name__ == "__main__":
    dimos = start(2)

    add_module = dimos.deploy(AddModule)
    ctrl_module = dimos.deploy(CtrlModule, remote_add=add_module.add)

    # we can inspect module I/O,
    #
    # in this case there are only two RPC functions available,
    # AddModule has a function named "add"
    print(add_module.io())
    # CtrlModule has a function  named "call"
    print(ctrl_module.io())

    # we can call the remote functions from the main loop
    assert add_module.add(1) == 2

    # or CtrlModule can also call AddModule's function
    assert ctrl_module.call(1) == 2
    dimos.shutdown()
