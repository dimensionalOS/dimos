from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Any

from demo_app.types import AlertEvent

logger = logging.getLogger(__name__)


class AudioAlert:
    def __init__(self, audio_file: str, runner: Any = None):
        self._audio_file = str(Path(audio_file))
        self._runner = runner

    async def play(self, event: AlertEvent) -> None:
        tasks = [asyncio.to_thread(self._play_local_blocking)]
        if self._runner is not None and hasattr(self._runner, "play_alert_on_robot"):
            tasks.append(asyncio.create_task(self._runner.play_alert_on_robot(self._audio_file)))
        await asyncio.gather(*tasks, return_exceptions=True)

    def _play_local_blocking(self) -> None:
        try:
            import playsound3

            playsound3.playsound(self._audio_file, block=True)
        except Exception as e:
            logger.warning("Local audio playback failed: %s", e)
