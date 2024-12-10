from datetime import timedelta
import sys
import os

# Add the parent directory of 'demos' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# -----

from dotenv import load_dotenv
load_dotenv()

from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler

from flask import Flask, Response, stream_with_context

from dimos.agents.agent import OpenAI_Agent 
from dimos.types.media_provider import VideoProviderExample
from dimos.web.edge_io import FlaskServer
from dimos.types.videostream import FrameProcessor
from dimos.types.videostream import StreamUtils

app = Flask(__name__)

def main():
    disposables = CompositeDisposable()

    # Create a frame processor to manipulate our video inputs
    processor = FrameProcessor()

    # Video provider setup
    my_video_provider = VideoProviderExample("Video File", video_source="rtsp://10.0.0.106:8080/h264.sdp") # "/app/dimos/multimedia/trimmed_video.mov")
    video_stream_obs = my_video_provider.video_capture_to_observable().pipe(
        ops.subscribe_on(ThreadPoolScheduler(6))
    )

    # Articficlally slow the stream (25fps ~ 40ms)
    slowed_video_stream_obs = StreamUtils.limit_emission_rate(video_stream_obs, time_delta=timedelta(milliseconds=40))

    # Process an edge detection stream
    edge_detection_stream_obs = processor.process_stream_edge_detection(slowed_video_stream_obs)

    # Process an optical flow stream
    optical_flow_stream_obs = processor.process_stream_optical_flow(slowed_video_stream_obs)

    # Dump streams to disk
    # Raw Frames
    slowed_video_stream_dump_obs = processor.process_stream_export_to_jpeg(slowed_video_stream_obs, suffix="raw")
    slowed_video_stream_dump_obs.subscribe(
        on_next=lambda result: None, # print(f"Slowed Stream Result: {result}"),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: print("Processing completed.")
    )

    # Edge Detection
    edge_detection_stream_dump_obs = processor.process_stream_export_to_jpeg(edge_detection_stream_obs, suffix="edge")
    edge_detection_stream_dump_obs.subscribe(
        on_next=lambda result: None, # print(f"Edge Detection Result: {result}"),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: print("Processing completed.")
    )

    # Optical Flow
    optical_flow_stream_dump_obs = processor.process_stream_export_to_jpeg(optical_flow_stream_obs, suffix="optical")
    optical_flow_stream_dump_obs.subscribe(
        on_next=lambda result: None, # print(f"Optical Flow Result: {result}"),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: print("Processing completed.")
    )

    # Local Optical Flow Threshold
    # TODO: Propogate up relevancy score from compute_optical_flow nested in process_stream_optical_flow

    # Agent Orchastrator (Qu.s Awareness, Temporality, Routing)
    # TODO: Expand

    # Agent 1
    my_agent = OpenAI_Agent("Agent 1", query="You are a robot. What do you see? Put a JSON with objects of what you see in the format {object, description}.")
    my_agent.subscribe_to_image_processing(slowed_video_stream_dump_obs)
    disposables.add(my_agent.disposables)

    # Agent 2
    my_agent_two = OpenAI_Agent("Agent 2", query="This is a visualization of dense optical flow. What movement(s) have occured? Put a JSON with mapped directions you see in the format {direction, probability, english_description}.")
    my_agent_two.subscribe_to_image_processing(optical_flow_stream_dump_obs)
    disposables.add(my_agent.disposables)

    # Create and start the Flask server
    # TODO: FIX
    flask_server = FlaskServer(None, frame_obs=slowed_video_stream_obs, 
                               frame_edge_obs=edge_detection_stream_obs, 
                               frame_optical_obs=optical_flow_stream_dump_obs)
    flask_server.run()

if __name__ == "__main__":
    main()

