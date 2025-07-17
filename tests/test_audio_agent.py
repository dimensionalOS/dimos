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

from dimos.stream.audio.utils import keepalive
from dimos.stream.audio.pipelines import tts, stt
from dimos.utils.threadpool import get_scheduler
from dimos.agents.agent import OpenAIAgent


def main():
    stt_node = stt()

    agent = OpenAIAgent(
        dev_name="UnitreeExecutionAgent",
        input_query_stream=stt_node.emit_text(),
        system_query="You are a helpful robot named daneel that does my bidding",
        pool_scheduler=get_scheduler(),
    )

    tts_node = tts()
    tts_node.consume_text(agent.get_response_observable())

    # Keep the main thread alive
    keepalive()


if __name__ == "__main__":
    main()
