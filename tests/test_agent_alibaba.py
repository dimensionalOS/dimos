import tests.test_header

import os
from dimos.agents.agent import OpenAIAgent
from openai import OpenAI
from dimos.stream.video_provider import VideoProvider
from dimos.utils.threadpool import get_scheduler
from dimos.agents.tokenizer.openai_impl import HuggingFace_Tokenizer
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills

# Initialize video stream
video_stream = VideoProvider(
    dev_name="VideoProvider",
    # video_source=f"{os.getcwd()}/assets/framecount.mp4",
    video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
    pool_scheduler=get_scheduler(),
).capture_video_as_observable(realtime=False, fps=1)

# Specify the OpenAI client for Alibaba
qwen_client = OpenAI(
                base_url='https://dashscope-intl.aliyuncs.com/compatible-mode/v1',
                api_key='<YOUR_ALIBABA_API_KEY>',
            )

# Initialize Unitree skills
myUnitreeSkills = MyUnitreeSkills()
myUnitreeSkills.initialize_skills()

# Initialize agent
agent = OpenAIAgent(
            dev_name="AlibabaExecutionAgent",
            openai_client=qwen_client,
            model_name="qwen2.5-vl-72b-instruct",
            tokenizer=HuggingFace_Tokenizer(model_name="Qwen/Qwen2.5-VL-72B-Instruct"),
            max_output_tokens_per_request=8192,
            input_video_stream=video_stream,
            # system_query="Tell me the number in the video. Find me the center of the number spotted, and print the coordinates to the console using an appropriate function call. Then provide me a deep history of the number in question and its significance in history. Additionally, tell me what model and version of language model you are.",
            system_query="Tell me about any objects seen. Print the coordinates for center of the objects seen to the console using an appropriate function call. Then provide me a deep history of the number in question and its significance in history. Additionally, tell me what model and version of language model you are.",
            skills=myUnitreeSkills,
        )

try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")