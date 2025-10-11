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

"""Simple vision agent module following exact DimOS patterns."""

import asyncio
import base64
import io
import threading
from typing import Optional

import numpy as np
from PIL import Image as PILImage

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger
from dimos.agents.modules.gateway import UnifiedGatewayClient
from reactivex.disposable import Disposable

logger = setup_logger(__file__)


class SimpleVisionAgentModule(Module):
    """Simple vision agent that can process images with text queries.

    This follows the exact pattern from working modules without any extras.
    """

    # Module I/O
    query_in: In[str] = None
    image_in: In[Image] = None
    response_out: Out[str] = None

    def __init__(
        self,
        model: str = "openai::gpt-4o-mini",
        system_prompt: str = None,
        temperature: float = 0.0,
        max_tokens: int = 4096,
    ):
        """Initialize the vision agent.

        Args:
            model: Model identifier (e.g., "openai::gpt-4o-mini")
            system_prompt: System prompt for the agent
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
        """
        super().__init__()

        self.model = model
        self.system_prompt = system_prompt or "You are a helpful vision AI assistant."
        self.temperature = temperature
        self.max_tokens = max_tokens

        # State
        self.gateway = None
        self._latest_image = None
        self._processing = False
        self._lock = threading.Lock()

    @rpc
    def start(self):
        """Initialize and start the agent."""
        super().start()

        logger.info(f"Starting simple vision agent with model: {self.model}")

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Subscribe to inputs
        if self.query_in:
            unsub = self.query_in.subscribe(self._handle_query)
            self._disposables.add(Disposable(unsub))

        if self.image_in:
            unsub = self.image_in.subscribe(self._handle_image)
            self._disposables.add(Disposable(unsub))

        logger.info("Simple vision agent started")

    @rpc
    def stop(self):
        logger.info("Stopping simple vision agent")
        if self.gateway:
            self.gateway.close()

        super().stop()

    def _handle_image(self, image: Image):
        """Handle incoming image."""
        logger.info(
            f"Received new image: {image.data.shape if hasattr(image, 'data') else 'unknown shape'}"
        )
        self._latest_image = image

    def _handle_query(self, query: str):
        """Handle text query."""
        with self._lock:
            if self._processing:
                logger.warning("Already processing, skipping query")
                return
            self._processing = True

        # Process in thread
        thread = threading.Thread(target=self._run_async_query, args=(query,))
        thread.daemon = True
        thread.start()

    def _run_async_query(self, query: str):
        """Run async query in new event loop."""
        asyncio.run(self._process_query(query))

    async def _process_query(self, query: str):
        """Process the query."""
        try:
            logger.info(f"Processing query: {query}")

            # Build messages
            messages = [{"role": "system", "content": self.system_prompt}]

            # Check if we have an image
            if self._latest_image:
                logger.info("Have latest image, encoding...")
                image_b64 = self._encode_image(self._latest_image)
                if image_b64:
                    logger.info(f"Image encoded successfully, size: {len(image_b64)} bytes")
                    # Add user message with image
                    if "anthropic" in self.model:
                        # Anthropic format
                        messages.append(
                            {
                                "role": "user",
                                "content": [
                                    {"type": "text", "text": query},
                                    {
                                        "type": "image",
                                        "source": {
                                            "type": "base64",
                                            "media_type": "image/jpeg",
                                            "data": image_b64,
                                        },
                                    },
                                ],
                            }
                        )
                    else:
                        # OpenAI format
                        messages.append(
                            {
                                "role": "user",
                                "content": [
                                    {"type": "text", "text": query},
                                    {
                                        "type": "image_url",
                                        "image_url": {
                                            "url": f"data:image/jpeg;base64,{image_b64}",
                                            "detail": "auto",
                                        },
                                    },
                                ],
                            }
                        )
                else:
                    # No image encoding, just text
                    logger.warning("Failed to encode image")
                    messages.append({"role": "user", "content": query})
            else:
                # No image at all
                logger.warning("No image available")
                messages.append({"role": "user", "content": query})

            # Make inference call
            response = await self.gateway.ainference(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                stream=False,
            )

            # Extract response
            message = response["choices"][0]["message"]
            content = message.get("content", "")

            # Emit response
            if self.response_out and content:
                self.response_out.publish(content)

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            import traceback

            traceback.print_exc()
            if self.response_out:
                self.response_out.publish(f"Error: {str(e)}")
        finally:
            with self._lock:
                self._processing = False

    def _encode_image(self, image: Image) -> Optional[str]:
        """Encode image to base64."""
        try:
            # Convert to numpy array if needed
            if hasattr(image, "data"):
                img_array = image.data
            else:
                img_array = np.array(image)

            # Convert to PIL Image
            pil_image = PILImage.fromarray(img_array)

            # Convert to RGB if needed
            if pil_image.mode != "RGB":
                pil_image = pil_image.convert("RGB")

            # Encode to base64
            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG")
            img_b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

            return img_b64

        except Exception as e:
            logger.error(f"Failed to encode image: {e}")
            return None
