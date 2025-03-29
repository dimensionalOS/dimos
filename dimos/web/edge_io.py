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

from reactivex.disposable import CompositeDisposable

class EdgeIO():
    def __init__(self, dev_name:str="NA", edge_type:str="Base"):
        self.dev_name = dev_name
        self.edge_type = edge_type
        self.disposables = CompositeDisposable()

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        self.disposables.dispose()
