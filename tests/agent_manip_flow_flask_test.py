"""
This module initializes and manages the video processing pipeline integrated with a web server.
It handles video capture, frame processing, and exposes the processed video streams via HTTP endpoints.
"""

import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# -----

# Standard library imports
import multiprocessing
from dotenv import load_dotenv

# Third-party imports
from flask import Flask
from reactivex import operators as ops
from reactivex import of, interval, zip
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler, ImmediateScheduler

# Local application imports
from dimos.agents.agent import PromptBuilder, OpenAIAgent 
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import VideoOperators as vops
from dimos.stream.video_provider import VideoProvider
from dimos.web.flask_server import FlaskServer

# Load environment variables
load_dotenv()

app = Flask(__name__)

def main():
    """
    Initializes and runs the video processing pipeline with web server output.

    This function orchestrates a video processing system that handles capture, processing,
    and visualization of video streams. It demonstrates parallel processing capabilities
    and various video manipulation techniques across multiple stages including capture
    and processing at different frame rates, edge detection, and optical flow analysis.

    Raises:
        RuntimeError: If video sources are unavailable or processing fails.
    """
    disposables = CompositeDisposable()

    processor = FrameProcessor(output_dir=f"{os.getcwd()}/assets/output/frames", delete_on_init=True)

    optimal_thread_count = multiprocessing.cpu_count()  # Gets number of CPU cores
    thread_pool_scheduler = ThreadPoolScheduler(optimal_thread_count)

    VIDEO_SOURCES = [
        f"{os.getcwd()}/assets/ldru.mp4",
        f"{os.getcwd()}/assets/ldru_480p.mp4",
        f"{os.getcwd()}/assets/trimmed_video_480p.mov",
        f"{os.getcwd()}/assets/video-f30-480p.mp4",
        f"{os.getcwd()}/assets/video.mov",
        "rtsp://192.168.50.207:8080/h264.sdp",
        "rtsp://10.0.0.106:8080/h264.sdp",
        f"{os.getcwd()}/assets/people_1080p_24fps.mp4"
    ]

    VIDEO_SOURCE_INDEX = 4

    my_video_provider = VideoProvider("Video File", video_source=VIDEO_SOURCES[VIDEO_SOURCE_INDEX])

    video_stream_obs = my_video_provider.capture_video_as_observable(fps=120).pipe(
        ops.subscribe_on(thread_pool_scheduler),
        # Move downstream operations to thread pool for parallel processing
        # Disabled: Evaluating performance impact
        # ops.observe_on(thread_pool_scheduler),
        # vops.with_jpeg_export(processor, suffix="raw"),
        vops.with_fps_sampling(fps=30),
        # vops.with_jpeg_export(processor, suffix="raw_slowed"),
    )

    edge_detection_stream_obs = processor.process_stream_edge_detection(video_stream_obs).pipe(
        # vops.with_jpeg_export(processor, suffix="edge"),
    )

    optical_flow_relevancy_stream_obs = processor.process_stream_optical_flow(video_stream_obs)

    optical_flow_stream_obs = optical_flow_relevancy_stream_obs.pipe(
        # ops.do_action(lambda result: print(f"Optical Flow Relevancy Score: {result[1]}")),
        # vops.with_optical_flow_filtering(threshold=2.0),
        # ops.do_action(lambda _: print(f"Optical Flow Passed Threshold.")),
        #vops.with_jpeg_export(processor, suffix="optical")
    )

    #
    # ====== Agent Orchastrator (Qu.s Awareness, Temporality, Routing) ====== 
    #

    # Observable that emits every 2 seconds
    secondly_emission = interval(2, scheduler=thread_pool_scheduler).pipe(
        ops.map(lambda x: f"Second {x+1}"),
        # ops.take(30)
    )

    # Agent 1
    my_agent = OpenAIAgent(
        "Agent 1", 
        query="You are a robot. What do you see? Put a JSON with objects of what you see in the format {object, description}.",
        json_mode=False
    )
    
    # Create an agent for each subset of questions that it would be theroized to handle. 
    # Set std. template/blueprints, and devs will add to that likely. 

    ai_1_obs = video_stream_obs.pipe(
        # vops.with_fps_sampling(fps=30),
        # ops.throttle_first(1),
        vops.with_jpeg_export(processor, suffix="open_ai_agent_1"),
        ops.take(30),
        ops.replay(buffer_size=30, scheduler=thread_pool_scheduler),
    )
    ai_1_obs.connect()

    ai_1_repeat_obs = ai_1_obs.pipe(ops.repeat())
    
    my_agent.subscribe_to_image_processing(ai_1_obs)
    disposables.add(my_agent.disposables)

    # Agent 2
    my_agent_two = OpenAIAgent(
        "Agent 2", 
        query="This is a visualization of dense optical flow. What movement(s) have occured? Put a JSON with mapped directions you see in the format {direction, probability, english_description}.",
        max_input_tokens_per_request=1000,
        max_output_tokens_per_request=300,
        json_mode=False,
        model_name="gpt-4o-2024-08-06",
    )
    
    ai_2_obs = optical_flow_stream_obs.pipe(
        # vops.with_fps_sampling(fps=30),
        # ops.throttle_first(1),
        vops.with_jpeg_export(processor, suffix="open_ai_agent_2"),
        ops.take(30),
        ops.replay(buffer_size=30, scheduler=thread_pool_scheduler),
    )
    ai_2_obs.connect()

    ai_2_repeat_obs = ai_2_obs.pipe(ops.repeat())


    # Combine emissions using zip
    ai_1_secondly_repeating_obs = zip(secondly_emission, ai_1_repeat_obs).pipe(
        # ops.do_action(lambda s: print(f"AI 1 - Emission Count: {s[0]}")),
        ops.map(lambda r: r[1]),
    )

    # Combine emissions using zip
    ai_2_secondly_repeating_obs = zip(secondly_emission, ai_2_repeat_obs).pipe(
        # ops.do_action(lambda s: print(f"AI 2 - Emission Count: {s[0]}")),
        ops.map(lambda r: r[1]),
    )

    
    my_agent_two.subscribe_to_image_processing(ai_2_obs)
    disposables.add(my_agent_two.disposables)

    #
    # ====== Create and start the Flask server ====== 
    #

    # Will be visible at http://[host]:[port]/video_feed/[key]
    flask_server = FlaskServer(
        # video_one=video_stream_obs,
        # edge_detection=edge_detection_stream_obs,
        # optical_flow=optical_flow_stream_obs,
        OpenAIAgent_1=ai_1_secondly_repeating_obs,
        OpenAIAgent_2=ai_2_secondly_repeating_obs,
    )
    
    flask_server.run(threaded=True)

if __name__ == "__main__":
    main()

