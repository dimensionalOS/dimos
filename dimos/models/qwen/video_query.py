"""Utility functions for one-off video frame queries using Qwen model."""

import os
from typing import Optional
from openai import OpenAI
from reactivex import Observable, operators as ops
from reactivex.subject import Subject

from dimos.agents.agent import OpenAIAgent
from dimos.agents.tokenizer.huggingface_tokenizer import HuggingFaceTokenizer
from dimos.utils.threadpool import get_scheduler
import json


def query_single_frame(
    video_observable: Observable,
    query: str,
    api_key: Optional[str] = None,
    model_name: str = "qwen2.5-vl-72b-instruct"
) -> Observable:
    """Process a single frame from a video observable with Qwen model.
    
    Args:
        video_observable: An observable that emits video frames
        query: The query to ask about the frame
        api_key: Alibaba API key. If None, will try to get from ALIBABA_API_KEY env var
        model_name: The Qwen model to use. Defaults to qwen2.5-vl-72b-instruct
        
    Returns:
        Observable: An observable that emits a single response string
        
    Example:
        ```python
        video_obs = video_provider.capture_video_as_observable()
        single_frame = video_obs.pipe(ops.take(1))
        response = query_single_frame(single_frame, "What objects do you see?")
        response.subscribe(print)
        ```
    """
    # Get API key from env if not provided
    api_key = api_key or os.getenv('ALIBABA_API_KEY')
    if not api_key:
        raise ValueError("Alibaba API key must be provided or set in ALIBABA_API_KEY environment variable")

    # Create Qwen client
    qwen_client = OpenAI(
        base_url='https://dashscope-intl.aliyuncs.com/compatible-mode/v1',
        api_key=api_key,
    )

    # Create response subject
    response_subject = Subject()
    
    # Create temporary agent for processing
    agent = OpenAIAgent(
        dev_name="QwenSingleFrameAgent",
        openai_client=qwen_client,
        model_name=model_name,
        tokenizer=HuggingFaceTokenizer(model_name=f"Qwen/{model_name}"),
        max_output_tokens_per_request=100,
        system_query=query,
        pool_scheduler=get_scheduler(),
    )

    # Take only first frame
    single_frame = video_observable.pipe(
        ops.take(1)
    )

    # Subscribe to frame processing and forward response to our subject
    agent.subscribe_to_image_processing(single_frame)
    
    # Forward agent responses to our response subject
    agent.get_response_observable().subscribe(
        on_next=lambda x: response_subject.on_next(x),
        on_error=lambda e: response_subject.on_error(e),
        on_completed=lambda: response_subject.on_completed()
    )

    # Clean up agent when response subject completes
    response_subject.subscribe(
        on_completed=lambda: agent.dispose_all()
    )

    return response_subject 


def get_bbox_from_qwen(
    video_stream: Observable,
    object_name: Optional[str] = None
) -> Optional[list]:
    """Get bounding box coordinates from Qwen for a specific object or any object.
    
    Args:
        video_stream: Observable video stream
        object_name: Optional name of object to detect
        
    Returns:
        bbox: Bounding box as [x1, y1, x2, y2] or None if no detection
    """
    prompt = (
        f"Look at this image and find the {object_name if object_name else 'most prominent object'}. Estimate the approximate height of the subject."
        "Return ONLY a JSON object with format: {'name': 'object_name', 'bbox': [x1, y1, x2, y2], 'size': height_in_meters} "
        "where x1,y1 is the top-left and x2,y2 is the bottom-right corner of the bounding box. If not found, return None."
    )

    response = query_single_frame(video_stream, prompt).pipe(ops.take(1)).run()

    try:
        # Extract JSON from response
        start_idx = response.find('{')
        end_idx = response.rfind('}') + 1
        if start_idx >= 0 and end_idx > start_idx:
            json_str = response[start_idx:end_idx]
            result = json.loads(json_str)
            
            # Extract and validate bbox
            if 'bbox' in result and len(result['bbox']) == 4:
                return result['bbox'], result['size']
    except Exception as e:
        print(f"Error parsing Qwen response: {e}")
        print(f"Raw response: {response}")

    return None