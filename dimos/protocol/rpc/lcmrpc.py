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

from dimos.constants import LCM_MAX_CHANNEL_NAME_LENGTH
from dimos.protocol.pubsub.lcmpubsub import PickleLCM, Topic
from dimos.protocol.rpc.pubsubrpc import PassThroughPubSubRPC
from dimos.utils.generic import short_id


class LCMRPC(PassThroughPubSubRPC, PickleLCM):  # type: ignore[type-arg]
    def topicgen(self, name: str, req_or_res: bool) -> Topic:
        suffix = "res" if req_or_res else "req"
        topic = f"/rpc/{name}/{suffix}"
        if len(topic) > LCM_MAX_CHANNEL_NAME_LENGTH:
            topic = f"/rpc/{short_id(name)}/{suffix}"
        return Topic(topic=topic)
