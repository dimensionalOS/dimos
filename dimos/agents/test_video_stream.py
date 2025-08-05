#!/usr/bin/env python3
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

"""Test video agent with real video stream using hot_latest."""

import asyncio
import os
from dotenv import load_dotenv
import pytest

from reactivex import operators as ops

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.agents.modules.simple_vision_agent import SimpleVisionAgentModule
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_video_stream")


class VideoStreamModule(Module):
    """Module that streams video continuously."""

    video_out: Out[Image] = None

    def __init__(self, video_path: str):
        super().__init__()
        self.video_path = video_path
        self._subscription = None

    @rpc
    def start(self):
        """Start streaming video."""
        # Use TimedSensorReplay to replay video frames
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Stream continuously at 10 FPS
        self._subscription = (
            video_replay.stream()
            .pipe(
                ops.sample(0.1),  # 10 FPS
            )
            .subscribe(
                on_next=lambda img: self.video_out.publish(img),
                on_error=lambda e: logger.error(f"Video stream error: {e}"),
                on_completed=lambda: logger.info("Video stream completed"),
            )
        )

        logger.info("Video streaming started at 10 FPS")

    @rpc
    def stop(self):
        """Stop streaming."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None


class VisionQueryAgent(Module):
    """Vision agent that uses hot_latest to get current frame when queried."""

    query_in: In[str] = None
    video_in: In[Image] = None
    response_out: Out[str] = None

    def __init__(self, model: str = "openai::gpt-4o-mini"):
        super().__init__()
        self.model = model
        self.agent = None
        self._hot_getter = None

    @rpc
    def start(self):
        """Start the agent."""
        from dimos.agents.modules.gateway import UnifiedGatewayClient

        logger.info(f"Starting vision query agent with model: {self.model}")

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Create hot_latest getter for video stream
        self._hot_getter = self.video_in.hot_latest()

        # Subscribe to queries
        self.query_in.subscribe(self._handle_query)

        logger.info("Vision query agent started")

    def _handle_query(self, query: str):
        """Handle query by getting latest frame."""
        logger.info(f"Received query: {query}")

        # Get the latest frame using hot_latest getter
        try:
            latest_frame = self._hot_getter()
        except Exception as e:
            logger.warning(f"No video frame available yet: {e}")
            self.response_out.publish("No video frame available yet.")
            return

        logger.info(
            f"Got latest frame: {latest_frame.data.shape if hasattr(latest_frame, 'data') else 'unknown'}"
        )

        # Process query with latest frame
        import threading

        thread = threading.Thread(
            target=lambda: asyncio.run(self._process_with_frame(query, latest_frame))
        )
        thread.daemon = True
        thread.start()

    async def _process_with_frame(self, query: str, frame: Image):
        """Process query with specific frame."""
        try:
            # Encode frame
            import base64
            import io
            import numpy as np
            from PIL import Image as PILImage

            # Get image data
            if hasattr(frame, "data"):
                img_array = frame.data
            else:
                img_array = np.array(frame)

            # Convert to PIL
            pil_image = PILImage.fromarray(img_array)
            if pil_image.mode != "RGB":
                pil_image = pil_image.convert("RGB")

            # Encode to base64
            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG", quality=85)
            img_b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

            # Build messages
            messages = [
                {
                    "role": "system",
                    "content": "You are a vision assistant. Describe what you see in the video frame.",
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": query},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"},
                        },
                    ],
                },
            ]

            # Make inference
            response = await self.gateway.ainference(
                model=self.model, messages=messages, temperature=0.0, max_tokens=500, stream=False
            )

            # Get response
            content = response["choices"][0]["message"]["content"]

            # Publish response
            self.response_out.publish(content)
            logger.info(f"Published response: {content[:100]}...")

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            import traceback

            traceback.print_exc()
            self.response_out.publish(f"Error: {str(e)}")


class ResponseCollector(Module):
    """Collect responses."""

    response_in: In[str] = None

    def __init__(self):
        super().__init__()
        self.responses = []

    @rpc
    def start(self):
        self.response_in.subscribe(self._on_response)

    def _on_response(self, resp: str):
        logger.info(f"Collected response: {resp[:100]}...")
        self.responses.append(resp)

    @rpc
    def get_responses(self):
        return self.responses


class QuerySender(Module):
    """Send queries at specific times."""

    query_out: Out[str] = None

    @rpc
    def send_query(self, query: str):
        self.query_out.publish(query)
        logger.info(f"Sent query: {query}")


@pytest.mark.module
@pytest.mark.asyncio
async def test_video_stream_agent():
    """Test vision agent with continuous video stream."""
    load_dotenv()
    pubsub.lcm.autoconf()

    logger.info("Testing vision agent with video stream and hot_latest...")
    dimos = core.start(4)

    try:
        # Get test video
        data_path = get_data("unitree_office_walk")
        video_path = os.path.join(data_path, "video")

        logger.info(f"Using video from: {video_path}")

        # Deploy modules
        video_stream = dimos.deploy(VideoStreamModule, video_path)
        video_stream.video_out.transport = core.LCMTransport("/vision/video", Image)

        query_sender = dimos.deploy(QuerySender)
        query_sender.query_out.transport = core.pLCMTransport("/vision/query")

        vision_agent = dimos.deploy(VisionQueryAgent, model="openai::gpt-4o-mini")
        vision_agent.response_out.transport = core.pLCMTransport("/vision/response")

        collector = dimos.deploy(ResponseCollector)

        # Connect modules
        vision_agent.video_in.connect(video_stream.video_out)
        vision_agent.query_in.connect(query_sender.query_out)
        collector.response_in.connect(vision_agent.response_out)

        # Start modules
        video_stream.start()
        vision_agent.start()
        collector.start()

        logger.info("All modules started, video streaming...")

        # Wait for video to stream some frames
        await asyncio.sleep(3)

        # Query 1: What do you see?
        logger.info("\n=== Query 1: General description ===")
        query_sender.send_query("What do you see in the current video frame?")
        await asyncio.sleep(4)

        # Wait a bit for video to progress
        await asyncio.sleep(2)

        # Query 2: More specific
        logger.info("\n=== Query 2: Specific details ===")
        query_sender.send_query("Describe any objects or furniture visible in the frame.")
        await asyncio.sleep(4)

        # Wait for video to progress more
        await asyncio.sleep(3)

        # Query 3: Changes
        logger.info("\n=== Query 3: Environment ===")
        query_sender.send_query("What kind of environment or room is this?")
        await asyncio.sleep(4)

        # Stop video stream
        video_stream.stop()

        # Get all responses
        responses = collector.get_responses()
        logger.info(f"\nCollected {len(responses)} responses:")
        for i, resp in enumerate(responses):
            logger.info(f"\nResponse {i + 1}: {resp}")

        # Verify we got responses
        assert len(responses) >= 3, f"Expected at least 3 responses, got {len(responses)}"

        # Verify responses describe actual scene
        all_responses = " ".join(responses).lower()
        assert any(
            word in all_responses
            for word in ["office", "room", "hallway", "corridor", "door", "wall", "floor"]
        ), "Responses should describe the office environment"

        logger.info("\n✅ Video stream agent test PASSED!")

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.module
@pytest.mark.asyncio
async def test_claude_video_stream():
    """Test Claude with video stream."""
    load_dotenv()

    if not os.getenv("ANTHROPIC_API_KEY"):
        logger.info("Skipping Claude - no API key")
        return

    pubsub.lcm.autoconf()

    logger.info("Testing Claude with video stream...")
    dimos = core.start(4)

    try:
        # Get test video
        data_path = get_data("unitree_office_walk")
        video_path = os.path.join(data_path, "video")

        # Deploy modules
        video_stream = dimos.deploy(VideoStreamModule, video_path)
        video_stream.video_out.transport = core.LCMTransport("/claude/video", Image)

        query_sender = dimos.deploy(QuerySender)
        query_sender.query_out.transport = core.pLCMTransport("/claude/query")

        vision_agent = dimos.deploy(VisionQueryAgent, model="anthropic::claude-3-haiku-20240307")
        vision_agent.response_out.transport = core.pLCMTransport("/claude/response")

        collector = dimos.deploy(ResponseCollector)

        # Connect modules
        vision_agent.video_in.connect(video_stream.video_out)
        vision_agent.query_in.connect(query_sender.query_out)
        collector.response_in.connect(vision_agent.response_out)

        # Start modules
        video_stream.start()
        vision_agent.start()
        collector.start()

        # Wait for streaming
        await asyncio.sleep(3)

        # Send query
        query_sender.send_query("Describe what you see in this video frame.")
        await asyncio.sleep(8)  # Claude needs more time

        # Stop stream
        video_stream.stop()

        # Check responses
        responses = collector.get_responses()
        assert len(responses) > 0, "Claude should respond"

        logger.info(f"Claude: {responses[0]}")
        logger.info("✅ Claude video stream test PASSED!")

    finally:
        dimos.close()
        dimos.shutdown()


if __name__ == "__main__":
    logger.info("Running video stream tests with hot_latest...")
    asyncio.run(test_video_stream_agent())
    print("\n" + "=" * 60 + "\n")
    asyncio.run(test_claude_video_stream())
