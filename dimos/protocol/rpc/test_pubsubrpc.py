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

import time

from dimos.protocol.rpc.pubsubrpc import PickleLCM


def test_basics():
    def remote_function(a: int, b: int):
        return a + b

    server = PickleLCM(autoconf=True)
    server.start()

    server.serve_rpc(remote_function, "add")

    client = PickleLCM(autoconf=True)
    client.start()
    msgs = []

    def receive_msg(response):
        msgs.append(response)
        print(f"Received response: {response}")

    client.call_cb("add", [1, 2], receive_msg)

    time.sleep(0.2)
    assert len(msgs) > 0
    server.stop()
    client.stop()
