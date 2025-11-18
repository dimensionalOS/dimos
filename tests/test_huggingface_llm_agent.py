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
import time

from dimos.agents.agent_huggingface_local import HuggingFaceLocalAgent
from dimos.stream.data_provider import QueryDataProvider

class HuggingFaceLLMAgentDemo:

    def __init__(self):
        self.robot_ip = None
        self.connection_method = None
        self.serial_number = None
        self.output_dir = None
        self._fetch_env_vars()

    def _fetch_env_vars(self):
        print("Fetching environment variables")

        def get_env_var(var_name, default=None, required=False):
            """Get environment variable with validation."""
            value = os.getenv(var_name, default)
            if required and not value:
                raise ValueError(f"{var_name} environment variable is required")
            return value

        self.robot_ip = get_env_var("ROBOT_IP", required=True)
        self.connection_method = get_env_var("CONN_TYPE")
        self.serial_number = get_env_var("SERIAL_NUMBER")
        self.output_dir = get_env_var(
            "ROS_OUTPUT_DIR", os.path.join(os.getcwd(), "assets/output/ros"))

    # -----

    def run_with_queries(self):
        # Initialize query stream
        query_provider = QueryDataProvider()

        # Create the skills available to the agent.
        # By default, this will create all skills in this class and make them available.

        print("Starting HuggingFace LLM Agent")

        # TESTING LOCAL AGENT
        self.HuggingFaceLLMAgent = HuggingFaceLocalAgent(
            dev_name="HuggingFaceLLMAgent",
            model_name= "Qwen/Qwen2.5-3B",
            agent_type="HF-LLM",
            input_query_stream=query_provider.data_stream,
            process_all_inputs=False,
            # output_dir=self.output_dir,
            # skills=skills_instance,
            # frame_processor=frame_processor,
        )

        # TESTING REMOTE AGENT
        # self.HuggingFaceLLMAgent = HuggingFaceRemoteAgent(
        #     dev_name="HuggingFaceLLMAgent",
        #     model_name= "Qwen/Qwen2.5-3B",
        #     agent_type="HF-LLM",
        #     input_query_stream=query_provider.data_stream,
        #     process_all_inputs=False,
        # )

        # Sample query to test the agent
        # self.HuggingFaceLLMAgent.stream_query("What is the capital of France?").subscribe(lambda x: print(x))

        # Start the query stream.
        # Queries will be pushed every 1 second, in a count from 100 to 5000.
        # This will cause listening agents to consume the queries and respond
        # to them via skill execution and provide 1-shot responses.
        query_provider.start_query_stream(
            query_template=
            "{query}; Denote the number at the beginning of this query before the semicolon as the 'reference number'. Provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            frequency=5,
            start_count=1,
            end_count=10000,
            step=1)
    # -----

    def stop(self):
        print("Stopping HuggingFace LLM Agent")
        self.HuggingFaceLLMAgent.dispose_all()


if __name__ == "__main__":
    myHuggingFaceLLMAgentDemo = HuggingFaceLLMAgentDemo()
    myHuggingFaceLLMAgentDemo.run_with_queries()

    # Keep the program running to allow the Unitree Agent Demo to operate continuously
    try:
        print("\nRunning HuggingFace LLM Agent Demo (Press Ctrl+C to stop)...")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping HuggingFace LLM Agent Demo")
        myHuggingFaceLLMAgentDemo.stop()
    except Exception as e:
        print(f"Error in main loop: {e}")
