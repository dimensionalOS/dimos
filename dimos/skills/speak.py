from dimos.skills.skills import AbstractSkill
from dimos.stream.audio.pipelines import tts
from pydantic import Field
from reactivex import Subject
from typing import Optional, Any

class Speak(AbstractSkill):
    """Speak text out loud to humans nearby or to other robots."""

    text: str = Field(..., description="Text to speak")

    def __init__(self, tts_node: Optional[Any] = None, **data):
        super().__init__(**data)
        self._tts_node = tts_node

    def __call__(self):
        text_subject = Subject()
        
        # Connect the Subject to the TTS node
        self._tts_node.consume_text(text_subject)
        
        # Emit the text to the Subject
        text_subject.on_next(self.text)
        
        return f"Spoke: {self.text} successfully"