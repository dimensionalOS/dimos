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

"""The exact message and module snippets used in the message-authoring walkthrough."""

from story_messages.story_msgs.msg import DeviceReading

from dimos.core.module import Module
from dimos.core.stream import In, Out


class ReadingProcessor(Module):
    raw: In[DeviceReading]
    reading: Out[DeviceReading]

    async def handle_raw(self, message: DeviceReading) -> None:
        self.reading.publish(
            DeviceReading(
                header=message.header,
                sequence=message.sequence,
                value=message.value + 1,
                label=message.label,
            )
        )
