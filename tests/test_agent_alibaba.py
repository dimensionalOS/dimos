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

import tests.test_header

import os
from dimos.agents.agent import OpenAIAgent
from openai import OpenAI
from dimos.stream.video_provider import VideoProvider
from dimos.utils.threadpool import get_scheduler
from dimos.agents.tokenizer.huggingface_tokenizer import HuggingFaceTokenizer
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
# Initialize video stream
video_stream = VideoProvider(
    dev_name="VideoProvider",
    # video_source=f"{os.getcwd()}/assets/framecount.mp4",
    video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
    pool_scheduler=get_scheduler(),
).capture_video_as_observable(realtime=False, fps=1)

# robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
#                   skills=MyUnitreeSkills())
# Specify the OpenAI client for Alibaba
qwen_client = OpenAI(
                base_url='https://dashscope-intl.aliyuncs.com/compatible-mode/v1',
                api_key=os.getenv('ALIBABA_API_KEY'),
            )

# Initialize Unitree skills
# myUnitreeSkills = MyUnitreeSkills()
# myUnitreeSkills.initialize_skills()

# Initialize agent
agent = OpenAIAgent(
            dev_name="AlibabaExecutionAgent",
            openai_client=qwen_client,
            model_name="qwen2.5-vl-72b-instruct",
            tokenizer=HuggingFaceTokenizer(model_name="Qwen/Qwen2.5-VL-72B-Instruct"),
            max_output_tokens_per_request=100,
            #input_video_stream=robot.get_ros_video_stream(),
            input_video_stream=video_stream,
            # system_query="Tell me the number in the video. Find me the center of the number spotted, and print the coordinates to the console using an appropriate function call. Then provide me a deep histeory of the number in question and its significance in history. Additionally, tell me what model and version of language model you are.",
            system_query="Tell me about any objects seen. Print the coordinates for center of the objects seen to the console using an appropriate function call.",
            # skills=robot.get_skills(),
        )


try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")