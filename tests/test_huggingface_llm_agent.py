import sys
import os
import time

from dimos.web.fastapi_server import FastAPIServer

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")
print(f"Current working directory: {os.getcwd()}")

# -----

from dimos.agents.agent import HuggingFaceLocalAgent, HuggingFaceRemoteAgent
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
        self.HuggingFaceAgent = HuggingFaceRemoteAgent(
            dev_name="HuggingFaceAgent",
            agent_type="HF-LLM",
            input_query_stream=query_provider.data_stream,
            process_all_inputs=False,
            # max_output_tokens_per_request=200,
            # output_dir=self.output_dir,
            # skills=skills_instance,
            # frame_processor=frame_processor,
        )

        # Sample query to test the agent
        # self.HuggingFaceLLMAgent.stream_query("What is the capital of France?").subscribe(lambda x: print(x))

        # Start the query stream.
        # Queries will be pushed every 1 second, in a count from 100 to 5000.
        # This will cause listening agents to consume the queries and respond
        # to them via skill execution and provide 1-shot responses.
        query_provider.start_query_stream(
            query_template=
            "X is: {query}. If X < 500, then say 'hi'. If X <= 500, but lower than 1000, then say 'hello'. If X is >= 1000 but lower than 2000, then say 'hi hello'. If X is >= 2000, but lower than 4600 then say 'hello?'. If X >= 4600, then say 'good day'. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            frequency=250,
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
