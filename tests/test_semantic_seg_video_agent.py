import tests.test_header

import cv2
import numpy as np
import os
import sys

from dimos.perception.semantic_seg import SemanticSegmentationStream
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.stream.video_operators import VideoOperators as MyVideoOps, Operators as MyOps
from dimos.stream.frame_processor import FrameProcessor
from reactivex import Subject, operators as RxOps
from dimos.agents.agent import OpenAIAgent
from dimos.utils.threadpool import get_scheduler

def main():
    # Unitree Go2 camera parameters at 1080p
    camera_params = {
        'resolution': (1920, 1080),  # 1080p resolution
        'focal_length': 3.2,  # mm
        'sensor_size': (4.8, 3.6)  # mm (1/4" sensor)
    }
    
    # Initialize video provider and segmentation stream
    from dimos.stream.video_provider import VideoProvider
    video_stream = VideoProvider(
        dev_name="UnitreeGo2",
        # video_source=f"{os.getcwd()}/assets/framecount.mp4",
        video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov",
        pool_scheduler=get_scheduler(),
    ).capture_video_as_observable(realtime=False, fps=5)
            
    seg_stream = SemanticSegmentationStream(enable_mono_depth=True, camera_params=camera_params, gt_depth_scale=512.0)
    
    # Create streams
    segmentation_stream = seg_stream.create_stream(
        video_stream.pipe(
            MyVideoOps.with_fps_sampling(fps=.5)
        )
    ) 
    # Throttling to slowdown SegmentationAgent calls 
    # TODO: Add Agent parameter to handle this called api_call_interval

    frame_processor = FrameProcessor(delete_on_init=True)
    seg_stream = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["viz_frame"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
        # MyVideoOps.with_jpeg_export(frame_processor=frame_processor, suffix="_frame_"), # debugging
    )

    depth_stream = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["depth_viz"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
    )

    object_stream = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["objects"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
        RxOps.map(lambda objects: "\n".join(
            f"Object {obj['object_id']}: {obj['label']} (confidence: {obj['prob']:.2f})" + 
            (f", depth: {obj['depth']:.2f}m" if 'depth' in obj else "")
            for obj in objects
        ) if objects else "No objects detected."),
    )

    text_query_stream = Subject()

    segmentation_agent = OpenAIAgent(
        dev_name="SemanticSegmentationAgent",
        model_name="gpt-4o",
        system_query="You are a helpful assistant that can help with tasks related to semantic segmentation. You will get a list of objects as input, reprint them out to me in a list. Your task will be to execute the MOVE() skill towards the box on the ground in front of you",
        input_query_stream=text_query_stream,
        input_data_stream=object_stream,
        input_video_stream=video_stream,
        process_all_inputs=False,
        pool_scheduler=get_scheduler(),
        # skills=robot.get_skills()
    )
    agent_response_stream = segmentation_agent.get_response_observable()

    print("Semantic segmentation visualization started. Press 'q' to exit.")

    streams = {
        "raw_stream": video_stream,
        "depth_stream": depth_stream,
        "seg_stream": seg_stream,
    }
    text_streams = {
        "object_stream": object_stream,
        "agent_response_stream": agent_response_stream,
    }

    try:
        fast_api_server = RobotWebInterface(port=5555, text_streams=text_streams, **streams)
        fast_api_server.query_stream.subscribe(lambda x: text_query_stream.on_next(x))    
        fast_api_server.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping...")
    finally:
        seg_stream.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    main()