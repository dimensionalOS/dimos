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

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out


class AliceModule(Module):
    greetings: In[str]
    response: Out[str]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.greetings.subscribe(self._on_greetings)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_greetings(self, greeting: str) -> None:
        self.response.publish(f"Hello {greeting} from Alice")
