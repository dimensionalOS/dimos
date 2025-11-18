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

from dimos.stream.data_provider import QueryDataProvider
import tests.test_header

import os
from dimos.stream.video_provider import VideoProvider
from dimos.utils.threadpool import get_scheduler
from dimos.agents.tokenizer.huggingface_tokenizer import HuggingFaceTokenizer
from dimos.agents.agent_huggingface_local import HuggingFaceLocalAgent
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills

# Initialize video stream
video_stream = VideoProvider(
    dev_name="VideoProvider",
    # video_source=f"{os.getcwd()}/assets/framecount.mp4",
    video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
    pool_scheduler=get_scheduler(),
).capture_video_as_observable(realtime=False, fps=1)

# Initialize Unitree skills
myUnitreeSkills = MyUnitreeSkills()
myUnitreeSkills.initialize_skills()

# Initialize query stream
query_provider = QueryDataProvider()

system_query = "You are a robot with the following functions. Move(), Reverse(), Left(), Right(), Stop(). Given the following user comands return ONLY the correct function."

# Initialize agent
agent = HuggingFaceLLMAgent = HuggingFaceLocalAgent(
    dev_name="HuggingFaceLLMAgent",
    model_name= "Qwen/Qwen2.5-3B",
    agent_type="HF-LLM",
    system_query=system_query,
    input_query_stream=query_provider.data_stream,
    process_all_inputs=False,
    max_input_tokens_per_request=250,
    max_output_tokens_per_request=20,
    # output_dir=self.output_dir,
    # skills=skills_instance,
    # frame_processor=frame_processor,
)

# Start the query stream.
# Queries will be pushed every 1 second, in a count from 100 to 5000.
# This will cause listening agents to consume the queries and respond
# to them via skill execution and provide 1-shot responses.
query_provider.start_query_stream(
    query_template=
    "{query}; User: travel forward by 10 meters",
    frequency=10,
    start_count=1,
    end_count=10000,
    step=1)

try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")