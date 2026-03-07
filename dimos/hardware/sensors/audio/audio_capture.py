# Copyright 2025-2026 Dimensional Inc.

import threading
import time
from dataclasses import dataclass
from functools import cache
from typing import Any

import numpy as np
import sounddevice as sd
from reactivex import Observable, create

from dimos.msgs.sensor_msgs.Audio import Audio


@dataclass
class AudioConfig:
    sample_rate: int = 16000
    channels: int = 1
    blocksize: int = 4096  # Larger blocksize for better efficiency with Whisper
    device: int | str | None = None


class AudioCapture:
    """Audio capture module using sounddevice."""

    def __init__(self, config: AudioConfig | None = None) -> None:
        self.config = config or AudioConfig()
        self._stream: sd.InputStream | None = None
        self._observer: Any = None
        self._lock = threading.Lock()

    @property
    @cache
    def audio_stream(self) -> Observable[Audio]:
        """Create an observable that starts/stops audio capture on subscription."""

        def subscribe(observer: Any, scheduler: Any = None) -> Any:
            with self._lock:
                self._observer = observer
                try:
                    self.start()
                except Exception as e:
                    observer.on_error(e)
                    return

            def dispose() -> None:
                with self._lock:
                    self._observer = None
                    self.stop()

            return dispose

        return create(subscribe)

    def start(self) -> None:
        if self._stream:
            return

        def callback(indata: np.ndarray, frames: int, time_info: Any, status: sd.CallbackFlags) -> None:
            if status:
                print(f"Audio capture status: {status}")
            
            # Use lock to ensure we don't emit to a disposed observer
            observer = self._observer
            if observer:
                # sounddevice gives us float32 by default with dtype='float32'
                data = indata.copy()
                if self.config.channels == 1:
                    data = data.flatten()

                ts = time.time()

                msg = Audio(
                    data=data,
                    sample_rate=self.config.sample_rate,
                    channels=self.config.channels,
                    ts=ts
                )
                observer.on_next(msg)

        try:
            self._stream = sd.InputStream(
                samplerate=self.config.sample_rate,
                blocksize=self.config.blocksize,
                device=self.config.device,
                channels=self.config.channels,
                callback=callback,
                dtype="float32",
            )
            self._stream.start()
        except Exception as e:
            if self._stream:
                self._stream.close()
                self._stream = None
            raise RuntimeError(f"Failed to start audio stream: {e}") from e

    def stop(self) -> None:
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
