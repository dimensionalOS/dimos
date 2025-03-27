import tests.test_header

import time
import os
from dimos.web.fastapi_server import FastAPIServer
from dimos.agents.agent import OpenAIAgent
from dimos.stream.data_provider import QueryDataProvider

print("Fetching environment variables")

def get_env_var(var_name, default=None, required=False):
    """Get environment variable with validation."""
    value = os.getenv(var_name, default)
    if required and not value:
        raise ValueError(f"{var_name} environment variable is required")
    return value

robot_ip = get_env_var("ROBOT_IP", required=True)
connection_method = get_env_var("CONN_TYPE")
serial_number = get_env_var("SERIAL_NUMBER")
output_dir = get_env_var(
    "ROS_OUTPUT_DIR", os.path.join(os.getcwd(), "assets/output/ros"))


# -----

def run_with_queries():
    # Initialize query stream
    query_provider = QueryDataProvider()

    print("Starting Unitree Perception Agent")
    my_UnitreePerceptionAgent = OpenAIAgent(
        dev_name="UnitreePerceptionAgent",
        agent_type="Perception",
        input_query_stream=query_provider.data_stream,
        output_dir=output_dir,
        # frame_processor=frame_processor,
    )

    # Start the query stream.
    # Queries will be pushed every 1 second, in a count from 100 to 5000.
    # This will cause listening agents to consume the queries and respond
    # to them via skill execution and provide 1-shot responses.
    query_provider.start_query_stream(
        query_template=
        "{query}; Denote the number at the beginning of this query before the semicolon as the 'reference number'. Provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
        frequency=0.01,
        start_count=1,
        end_count=10000,
        step=1)


def run_with_test_video():
    # Initialize test video stream
    from dimos.stream.video_provider import VideoProvider
    my_video_stream = VideoProvider(
        dev_name="UnitreeGo2",
        video_source=f"{os.getcwd()}/assets/framecount.mp4"
    ).capture_video_as_observable()

    print("Starting Unitree Perception Agent (Test Video)")
    my_UnitreePerceptionAgent = OpenAIAgent(
        dev_name="UnitreePerceptionAgent",
        agent_type="Perception",
        input_video_stream=my_video_stream,
        query=
        "Denote the number you see in the image as the 'reference number'. Only provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
        image_detail="high",
        # frame_processor=frame_processor,
    )


def run_with_queries_and_fast_api():
    # Initialize test video stream
    from dimos.stream.video_provider import VideoProvider
    my_video_stream = VideoProvider(
        dev_name="UnitreeGo2",
        video_source=f"{os.getcwd()}/assets/framecount.mp4"
    ).capture_video_as_observable()

    # Will be visible at http://[host]:[port]/video_feed/[key]
    streams = {
        "unitree_video": my_video_stream,
    }
    fast_api_server = FastAPIServer(port=5555, **streams)

    print("Starting Unitree Perception Agent")
    my_UnitreeQueryPerceptionAgent = OpenAIAgent(
        dev_name="UnitreeQueryPerceptionAgent",
        agent_type="Perception",
        input_query_stream=fast_api_server.query_stream,
    )

    # Run the FastAPI server (this will block)
    fast_api_server.run()

# endregion

def run_new_rx():
    print("Starting new RX")

    # Initialize test video stream
    from dimos.stream.video_provider import VideoProvider
    my_video_stream = VideoProvider(
        dev_name="UnitreeGo2",
        video_source=f"{os.getcwd()}/assets/framecount.mp4"
    ).capture_video_as_observable()

    # Will be visible at http://[host]:[port]/video_feed/[key]
    streams = {
        "unitree_video": my_video_stream,
    }
    fast_api_server = FastAPIServer(port=5555, **streams)
    
    fast_api_server.query_stream.subscribe(lambda x: print(f"Query: {x}"))

    from reactivex import operators as ops
    from dimos.utils.threadpool import get_scheduler
    print("Starting Unitree Perception Agent")
    my_UnitreeQueryPerceptionAgent = OpenAIAgent(
        dev_name="UnitreeQueryPerceptionAgent",
        agent_type="Perception",
        input_query_stream=fast_api_server.query_stream,
        input_video_stream=my_video_stream,
        process_all_inputs=True,
        pool_scheduler=get_scheduler(),
    )

    # Run the FastAPI server (this will block)
    fast_api_server.run()


# region Main
if __name__ == "__main__":
    # run_with_queries()
    # run_with_test_video()
    # run_with_queries_and_fast_api()
    run_new_rx()

    # Keep the program running to allow the Unitree Agent Demo to operate continuously
    try:
        print("\nRunning Unitree Agent Demo (Press Ctrl+C to stop)...")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        exit()
    except Exception as e:
        print(f"Error in main loop: {e}")
# endregion