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

from dask.distributed import Client, LocalCluster, Queue, Worker, get_client, get_worker


class Actor:
    def __init__(self, worker: Worker = None, client: Client = None):
        if worker is None and client is None:
            try:
                self.worker = get_worker()
            except Exception:
                self.client = get_client()

    @property
    def dask(self):
        if self.client:
            return self.client
        elif self.worker:
            return self.worker

    def pub(self, topic, msg):
        self.dask.log_event(topic, msg)

    def sub(self, topic, handler):
        self.dask.subscribe_topic(topic, handler)
        return lambda: self.unsub(topic, handler)

    def unsub(self, topic, handler):
        self.dask.unsubscribe_topic(topic, handler)
