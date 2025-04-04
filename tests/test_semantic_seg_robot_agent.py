import cv2
import numpy as np
import os
import sys

from dimos.stream.video_provider import VideoProvider
from dimos.perception.semantic_seg import SemanticSegmentationStream
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
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
    
    robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                        ros_control=UnitreeROSControl(),
                        skills=MyUnitreeSkills())
            
    seg_stream = SemanticSegmentationStream(enable_mono_depth=True, camera_params=camera_params, gt_depth_scale=512.0)
    
    # Create streams
    video_stream = robot.get_ros_video_stream(fps=5)
    segmentation_stream = seg_stream.create_stream(
        video_stream.pipe(
            MyVideoOps.with_fps_sampling(fps=.5)
        ) 
    )
    # Throttling to slowdown SegmentationAgent calls 
    # TODO: add Agent parameter to handle this called api_call_interval

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
    
    # Combine text query with latest object data when a new text query arrives
    enriched_query_stream = text_query_stream.pipe(
        RxOps.with_latest_from(object_stream),
        RxOps.map(lambda combined: {
            "query": combined[0],
            "objects": combined[1] if len(combined) > 1 else "No object data available"
        }),
        RxOps.map(lambda data: f"{data['query']}\n\nCurrent objects detected:\n{data['objects']}"),
        RxOps.do_action(lambda x: print(f"\033[34mEnriched query: {x.split(chr(10))[0]}\033[0m") or 
                                [print(f"\033[34m{line}\033[0m") for line in x.split(chr(10))[1:]]),
    )

    segmentation_agent = OpenAIAgent(
        dev_name="SemanticSegmentationAgent",
        model_name="gpt-4o",
        system_query="You are a helpful assistant that can control a virtual robot with semantic segmentation / distnace data as a guide. Only output skill calls, no other text",
        input_query_stream=enriched_query_stream,
        process_all_inputs=False,
        pool_scheduler=get_scheduler(),
        skills=robot.get_skills()
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
        "enriched_query_stream": enriched_query_stream,
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