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

from dimos.agents.agent_ctransformers_gguf import CTransformersGGUFAgent
from dimos.stream.data_provider import QueryDataProvider

class CTransformersGGUFAgentDemo:

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

        print("Starting CTransformers GGUF Agent")

        # Check for NVIDIA GPU availability
        import torch
        device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda":
            print(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("GPU not available, using CPU")

        agent = CTransformersGGUFAgent(
            dev_name="GGUF-Agent",
            model_name="TheBloke/Llama-2-7B-GGUF",
            model_file="llama-2-7b.Q4_K_M.gguf",
            model_type="llama",
            gpu_layers=50,
            max_input_tokens_per_request=250,
            max_output_tokens_per_request=10,
        )

        test_query = "User: Move forward 10m"
        
        # response = agent.query(test_query)
        # print(response)
        agent.run_observable_query(test_query).subscribe(
            on_next=lambda response: print(f"One-off query response: {response}"),
            on_error=lambda error: print(f"Error: {error}"),
            on_completed=lambda: print("Query completed")
        )

    def stop(self):
        print("Stopping CTransformers GGUF Agent")
        self.CTransformersGGUFAgent.dispose_all()


if __name__ == "__main__":
    myCTransformersGGUFAgentDemo = CTransformersGGUFAgentDemo()
    myCTransformersGGUFAgentDemo.run_with_queries()

    # Keep the program running to allow the Unitree Agent Demo to operate continuously
    try:
        print("\nRunning HuggingFace LLM Agent Demo (Press Ctrl+C to stop)...")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping CTransformers GGUF Agent Demo")
        myCTransformersGGUFAgentDemo.stop()
    except Exception as e:
        print(f"Error in main loop: {e}")
