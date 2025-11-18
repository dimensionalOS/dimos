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

from dimos.agents.agent_ctransformers_gguf import CTransformersGGUFAgent

system_query = "You are a robot with the following functions. Move(), Reverse(), Left(), Right(), Stop(). Given the following user comands return the correct function."

# Initialize agent
agent = CTransformersGGUFAgent(
    dev_name="GGUF-Agent",
    model_name="TheBloke/Llama-2-7B-GGUF",
    model_file="llama-2-7b.Q4_K_M.gguf",
    model_type="llama",
    system_query=system_query,
    gpu_layers=50,
    max_input_tokens_per_request=250,
    max_output_tokens_per_request=10,
)

test_query = "User: Travel forward 10 meters"

agent.run_observable_query(test_query).subscribe(
    on_next=lambda response: print(f"One-off query response: {response}"),
    on_error=lambda error: print(f"Error: {error}"),
    on_completed=lambda: print("Query completed")
)

try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")