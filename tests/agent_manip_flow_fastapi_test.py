"""
This module initializes and manages the video processing pipeline integrated with a web server.
It handles video capture, frame processing, and exposes the processed video streams via HTTP endpoints.
"""

import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----

# Standard library imports
import multiprocessing
from dotenv import load_dotenv

# Third-party imports
from fastapi import FastAPI
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler, ImmediateScheduler

# Local application imports
from dimos.agents.agent import OpenAI_Agent 
from dimos.types.videostream import FrameProcessor, VideoOperators as vops
from dimos.types.media_provider import VideoProviderExample
from dimos.web.fastapi_server import FastAPIServer

# Load environment variables
load_dotenv()

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

    processor = FrameProcessor(output_dir="/app/assets/frames", delete_on_init=True)

    optimal_thread_count = multiprocessing.cpu_count()  # Gets number of CPU cores
    thread_pool_scheduler = ThreadPoolScheduler(optimal_thread_count)

    VIDEO_SOURCES = [
        "/app/assets/ldru.mp4",
        "/app/assets/ldru_480p.mp4",
        "/app/assets/trimmed_video_480p.mov",
        "/app/assets/video-f30-480p.mp4",
        "rtsp://192.168.50.207:8080/h264.sdp",
        "rtsp://10.0.0.106:8080/h264.sdp"
    ]

    VIDEO_SOURCE_INDEX = 3
    VIDEO_SOURCE_INDEX_2 = 2

    my_video_provider = VideoProviderExample("Video File", video_source=VIDEO_SOURCES[VIDEO_SOURCE_INDEX])
    my_video_provider_2 = VideoProviderExample("Video File 2", video_source=VIDEO_SOURCES[VIDEO_SOURCE_INDEX_2])

    video_stream_obs = my_video_provider.video_capture_to_observable(fps=120).pipe(
        ops.subscribe_on(thread_pool_scheduler),
        # Move downstream operations to thread pool for parallel processing
        # Disabled: Evaluating performance impact
        # ops.observe_on(thread_pool_scheduler),
        vops.with_jpeg_export(processor, suffix="raw"),
        vops.with_fps_sampling(fps=30),
        vops.with_jpeg_export(processor, suffix="raw_slowed"),
    )

    video_stream_obs_2 = my_video_provider_2.video_capture_to_observable(fps=120).pipe(
        ops.subscribe_on(thread_pool_scheduler),
        # Move downstream operations to thread pool for parallel processing
        # Disabled: Evaluating performance impact
        # ops.observe_on(thread_pool_scheduler),
        vops.with_jpeg_export(processor, suffix="raw_2"),
        vops.with_fps_sampling(fps=30),
        vops.with_jpeg_export(processor, suffix="raw_2_slowed"),
    )

    edge_detection_stream_obs = processor.process_stream_edge_detection(video_stream_obs).pipe(
        vops.with_jpeg_export(processor, suffix="edge"),
    )

    optical_flow_relevancy_stream_obs = processor.process_stream_optical_flow_with_relevancy(video_stream_obs)

    optical_flow_stream_obs = optical_flow_relevancy_stream_obs.pipe(
        ops.do_action(lambda result: print(f"Optical Flow Relevancy Score: {result[1]}")),
        vops.with_optical_flow_filtering(threshold=2.0),
        ops.do_action(lambda _: print(f"Optical Flow Passed Threshold.")),
        vops.with_jpeg_export(processor, suffix="optical")
    )

    #
    # ====== Agent Orchastrator (Qu.s Awareness, Temporality, Routing) ====== 
    #

    # Agent 1
    # my_agent = OpenAI_Agent(
    #     "Agent 1", 
    #     query="You are a robot. What do you see? Put a JSON with objects of what you see in the format {object, description}.")
    # my_agent.subscribe_to_image_processing(slowed_video_stream_obs)
    # disposables.add(my_agent.disposables)

    # # Agent 2
    # my_agent_two = OpenAI_Agent(
    #     "Agent 2", 
    #     query="This is a visualization of dense optical flow. What movement(s) have occured? Put a JSON with mapped directions you see in the format {direction, probability, english_description}.")
    # my_agent_two.subscribe_to_image_processing(optical_flow_stream_obs)
    # disposables.add(my_agent_two.disposables)

    #
    # ====== Create and start the FastAPI server ====== 
    #
    
    # Will be visible at http://[host]:[port]/video_feed/[key]
    streams = {
        "video_one": video_stream_obs,
        "video_two": video_stream_obs_2,
        "edge_detection": edge_detection_stream_obs,
        "optical_flow": optical_flow_stream_obs,
    }
    fast_api_server = FastAPIServer(port=5555, **streams)
    fast_api_server.run()

if __name__ == "__main__":
    main()

