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

"""Implementation runs only inside the package-owned runtime environment."""

from dimos_package_python.module import IsolatedObserver, record

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.Twist import Twist


class ObserverRuntime(IsolatedObserver):
    @rpc
    def start(self) -> None:
        super().start()
        self.data.subscribe(self._record)

    def _record(self, message: Twist) -> None:
        record(message, "isolated")
