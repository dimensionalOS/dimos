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

"""AgentMessage type for multimodal agent communication."""

from dataclasses import dataclass, field
import time

from dimos.agents_deprecated.agent_types import AgentImage
from dimos.msgs.sensor_msgs.Image import Image


@dataclass
class AgentMessage:
    """Message type for agent communication with text and images.

    This type supports multimodal messages containing both text strings
    and AgentImage objects (base64 encoded) for vision-enabled agents.

    The messages field contains multiple text strings that will be combined
    into a single message when sent to the LLM.
    """

    messages: list[str] = field(default_factory=list)
    images: list[AgentImage] = field(default_factory=list)
    sender_id: str | None = None
    timestamp: float = field(default_factory=time.time)

    def add_text(self, text: str) -> None:
        """Add a text message."""
        if text:  # Only add non-empty text
            self.messages.append(text)

    def add_image(self, image: Image | AgentImage) -> None:
        """Add an image. Converts Image to AgentImage if needed."""
        if isinstance(image, Image):
            # Convert to AgentImage
            agent_image = AgentImage(
                base64_jpeg=image.agent_encode(),  # type: ignore[arg-type]
                width=image.width,
                height=image.height,
                metadata={"format": image.format.value, "frame_id": image.frame_id},
            )
            self.images.append(agent_image)
        elif isinstance(image, AgentImage):
            self.images.append(image)
        else:
            raise TypeError(f"Expected Image or AgentImage, got {type(image)}")

    def has_text(self) -> bool:
        """Check if message contains text."""
        # Check if we have any non-empty messages
        return any(msg for msg in self.messages if msg)

    def has_images(self) -> bool:
        """Check if message contains images."""
        return len(self.images) > 0

    def is_multimodal(self) -> bool:
        """Check if message contains both text and images."""
        return self.has_text() and self.has_images()

    def get_primary_text(self) -> str | None:
        """Get the first text message, if any."""
        return self.messages[0] if self.messages else None

    def get_primary_image(self) -> AgentImage | None:
        """Get the first image, if any."""
        return self.images[0] if self.images else None

    def get_combined_text(self) -> str:
        """Get all text messages combined into a single string."""
        # Filter out any empty strings and join
        return " ".join(msg for msg in self.messages if msg)

    def clear(self) -> None:
        """Clear all content."""
        self.messages.clear()
        self.images.clear()

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"AgentMessage("
            f"texts={len(self.messages)}, "
            f"images={len(self.images)}, "
            f"sender='{self.sender_id}', "
            f"timestamp={self.timestamp})"
        )
