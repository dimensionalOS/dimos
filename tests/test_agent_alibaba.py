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
import reactivex
from dimos.models.qwen.video_query import query_single_frame

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
# agent = OpenAIAgent(
#             dev_name="AlibabaExecutionAgent",
#             openai_client=qwen_client,
#             model_name="qwen2.5-vl-72b-instruct",
#             tokenizer=HuggingFaceTokenizer(model_name="Qwen/Qwen2.5-VL-72B-Instruct"),
#             max_output_tokens_per_request=100,
#             #input_video_stream=robot.get_ros_video_stream(),
#             input_video_stream=video_stream,
#             # system_query="Tell me the number in the video. Find me the center of the number spotted, and print the coordinates to the console using an appropriate function call. Then provide me a deep histeory of the number in question and its significance in history. Additionally, tell me what model and version of language model you are.",
#             system_query="Tell me about any objects seen. Print the coordinates for center of the objects seen to the console using an appropriate function call.",
#             # skills=robot.get_skills(),
#         )

def test_single_frame():
    # Create video provider for a single frame
    video_provider = VideoProvider(
        dev_name="SingleFrameProvider",
        video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
        pool_scheduler=get_scheduler(),
    )
    
    # Get video observable with 1 FPS
    video_obs = video_provider.capture_video_as_observable(realtime=False, fps=1)
    
    # Take just the first frame using take(1)
    single_frame_obs = video_obs.pipe(
        reactivex.operators.take(1)  # Only take the first frame
    )
    
    # Create agent for single frame processing
    single_frame_agent = OpenAIAgent(
        dev_name="SingleFrameAgent",
        openai_client=qwen_client,
        model_name="qwen2.5-vl-72b-instruct",
        tokenizer=HuggingFaceTokenizer(model_name="Qwen/Qwen2.5-VL-72B-Instruct"),
        max_output_tokens_per_request=100,
        system_query="Describe what you see in this single frame.",
    )
    
    # Process the single frame
    print("Processing single frame...")
    single_frame_agent.subscribe_to_image_processing(single_frame_obs)
    
    # Wait for processing to complete
    try:
        input("Press Enter after frame is processed...")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        single_frame_agent.dispose_all()
        video_provider.dispose_all()

def test_qwen_single_frame():
    # Create video provider
    video_provider = VideoProvider(
        dev_name="SingleFrameProvider",
        video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
        pool_scheduler=get_scheduler(),
    )
    
    # Get video observable
    video_obs = video_provider.capture_video_as_observable(realtime=False, fps=1)
    
    print("Processing single frame with Qwen...")
    # Use the new utility function
    response = query_single_frame(
        video_obs,
        "What objects do you see in this frame? List them in bullet points."
    )
    
    # Subscribe to get the response
    response.subscribe(
        on_next=lambda x: print(f"\nQwen response:\n{x}"),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: video_provider.dispose_all()
    )
    
    # Wait for processing to complete
    try:
        input("Press Enter after frame is processed...")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        video_provider.dispose_all()

if __name__ == "__main__":
    # test_single_frame()  # Comment out old test
    test_qwen_single_frame()  # Run new test

try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")