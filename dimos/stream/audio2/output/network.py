#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Network audio streaming output using GStreamer RTP."""

from typing import Optional

import gi
from pydantic import Field

gi.require_version("Gst", "1.0")

from reactivex import Observable

from dimos.stream.audio2.base import GStreamerSinkBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.output.network")


class NetworkOutputConfig(GStreamerNodeConfig):
    """Configuration for network audio output."""

    host: str = Field(default="127.0.0.1", description="Target host IP address")
    port: int = Field(default=5002, description="Target UDP port for RTP stream")
    codec: str = Field(default="opus", description="Audio codec (opus, vorbis, mp3, etc.)")
    bitrate: int = Field(default=128000, description="Audio bitrate in bits per second")
    buffer_time: int = Field(
        default=40960, description="UDP socket buffer size in bytes (default: 40KB)"
    )
    queue_size: int = Field(
        default=10,
        description="Number of buffers to queue before payloader (default: 10 for ~200ms at 20ms frames)",
    )


class NetworkOutputNode(GStreamerSinkBase):
    """GStreamer-based network audio output that streams via RTP.

    This sends audio to a remote server using RTP/UDP. Compatible with
    the gstreamer.sh server script that listens on UDP port 5002.
    """

    def __init__(self, config: NetworkOutputConfig):
        super().__init__(config)
        self.config: NetworkOutputConfig = config

    def _create_pipeline(self):
        """Create pipeline and set up system clock for timestamp-based streaming."""
        super()._create_pipeline()

        # Use system clock so udpsink can sync to timestamps
        # This allows sync=True to work properly and send packets at real-time rate
        from gi.repository import Gst

        clock = Gst.SystemClock.obtain()
        self._pipeline.use_clock(clock)
        logger.info(f"{self._get_sink_name()}: Using system clock for timestamp-based streaming")

    def _configure_sink(self):
        """Configure the udpsink after pipeline creation."""
        # Use sync=False to send packets as they arrive, not paced by timestamps
        # This is necessary for concatenated streams with discontinuous timestamps
        # The receiver handles playback timing
        from gi.repository import Gst

        # Get udpsink by iterating elements
        it = self._pipeline.iterate_elements()
        result, elem = it.next()
        while result == Gst.IteratorResult.OK:
            if elem.get_factory().get_name() == "udpsink":
                elem.set_property("sync", False)
                elem.set_property("async", False)
                logger.info(f"Set sync=False, async=False on udpsink (sends as fast as available)")
                break
            result, elem = it.next()

    def _get_pipeline_string(self) -> str:
        """Get the network output pipeline string.

        Pipeline: queue → decodebin → audioconvert → audioresample → encoder → queue → rtppay → udpsink
        """
        import random

        codec = self.config.codec.lower()
        host = self.config.host
        port = self.config.port
        bitrate = self.config.bitrate

        # Generate random SSRC so receiver can distinguish between different streams
        # This allows jitterbuffer to reset timing when a new stream starts
        ssrc = random.randint(0, 0xFFFFFFFF)

        if codec == "opus":
            # Opus codec (recommended for low latency and quality)
            # Ensure bitrate is set for consistent quality
            encoder = f"opusenc bitrate={bitrate} frame-size=20"
            payloader = f"rtpopuspay ssrc={ssrc}"
        elif codec == "vorbis":
            # Vorbis codec
            encoder = f"vorbisenc quality=0.5"
            payloader = f"rtpvorbispay ssrc={ssrc}"
        elif codec == "pcm" or codec == "raw":
            # Raw PCM audio (L16)
            encoder = "audioconvert"
            payloader = f"rtpL16pay ssrc={ssrc}"
        else:
            raise ValueError(f"Unsupported codec: {codec}. Use 'opus', 'vorbis', or 'pcm'")

        # Add minimal buffering for smooth playback without excess latency
        # - First queue: unlimited size to handle decoding bursts
        # - Second queue: small buffer (default 10 frames = ~200ms for 20ms Opus frames)
        pipeline = (
            f"queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! "
            f"decodebin ! "
            f"audioconvert ! audioresample ! "
            f"{encoder} ! "
            f"queue max-size-buffers={self.config.queue_size} ! "
            f"{payloader} ! "
            f"udpsink host={host} port={port} buffer-size={self.config.buffer_time}"
        )

        logger.info(f"NetworkOutput: Using SSRC={ssrc} for new stream")

        return pipeline

    def _get_sink_name(self) -> str:
        """Get a descriptive name."""
        return f"NetworkOutput[{self.config.host}:{self.config.port}]"


def network_output(
    host: str = "127.0.0.1",
    port: int = 5002,
    codec: str = "opus",
    bitrate: int = 128000,
    buffer_time: int = 40960,
    queue_size: int = 10,
    **kwargs,
):
    """Create a network audio output that streams via RTP/UDP.

    This creates an audio sink that sends audio to a remote server using
    RTP over UDP. Compatible with the gstreamer.sh server script.

    Args:
        host: Target host IP address (default: "127.0.0.1")
        port: Target UDP port (default: 5002)
        codec: Audio codec - "opus" (recommended), "vorbis", or "pcm" (default: "opus")
        bitrate: Audio bitrate in bits per second (default: 128000)
        buffer_time: UDP socket buffer size in bytes (default: 40960 = 40KB)
        queue_size: Number of buffers to queue before sending (default: 10 = ~200ms at 20ms frames)
        **kwargs: Additional arguments passed to NetworkOutputConfig:
            - output: Output audio specification (default: auto-detect)
            - properties: GStreamer element properties

    Returns:
        An operator function that consumes AudioEvents

    Examples:
        # Stream to remote server with Opus codec
        microphone().pipe(
            network_output(host="192.168.1.100", port=5002)
        ).run()

        # Stream with higher bitrate for better quality
        file_input("audio.mp3").pipe(
            network_output(host="10.0.0.5", bitrate=256000)
        ).run()

        # Lower latency for short audio clips (less buffering)
        signal(frequency=440, duration=0.5).pipe(
            network_output(queue_size=2)  # Only 40ms buffer
        ).run()

        # Higher stability for unreliable networks (more buffering)
        microphone().pipe(
            network_output(queue_size=25)  # 500ms buffer
        ).run()

        # Stream raw PCM (low latency, high bandwidth)
        signal(frequency=440).pipe(
            network_output(codec="pcm")
        ).run()

        # Local streaming for testing
        microphone().pipe(
            network_output()  # Defaults to localhost:5002
        ).run()
    """
    import threading

    config = NetworkOutputConfig(
        host=host,
        port=port,
        codec=codec,
        bitrate=bitrate,
        buffer_time=buffer_time,
        queue_size=queue_size,
        **kwargs,
    )
    node = NetworkOutputNode(config)

    # Return a dual-purpose object that can be used as operator or sink
    class NetworkOutputOperator:
        """Can be used as both an operator and a sink."""

        def __init__(self, sink_node):
            self._sink = sink_node

        def __call__(self, source):
            """Act as an operator for pipe()."""
            from reactivex import create

            def subscribe(observer, scheduler=None):
                # Wrap sink callbacks
                def on_next_wrapper(value):
                    self._sink.on_next(value)
                    observer.on_next(value)

                def on_error_wrapper(error):
                    self._sink.on_error(error)
                    observer.on_error(error)

                def on_completed_wrapper():
                    logger.info("NetworkOutputOperator: on_completed received from source")
                    self._sink.on_completed()

                    # Wait for the network sink to finish streaming
                    # With sync=True, packets are sent at real-time rate
                    # The sink will set _is_playing=False when EOS completes
                    import time

                    max_wait = 5.0  # 5 seconds should be plenty for EOS to process
                    start = time.time()
                    logger.info(
                        f"NetworkOutputOperator: Waiting for network sink to finish (is_playing={self._sink._is_playing})"
                    )
                    while self._sink._is_playing:
                        if time.time() - start > max_wait:
                            logger.warning(
                                f"NetworkOutputOperator: Timeout waiting for EOS, forcing stop()"
                            )
                            # Force stop if EOS didn't trigger it (e.g., mainloop died)
                            self._sink.stop()
                            break
                        time.sleep(0.05)

                    elapsed = time.time() - start
                    logger.info(
                        f"NetworkOutputOperator: Network sink finished after {elapsed:.2f}s"
                    )
                    observer.on_completed()

                # Subscribe to source
                return source.subscribe(
                    on_next=on_next_wrapper,
                    on_error=on_error_wrapper,
                    on_completed=on_completed_wrapper,
                    scheduler=scheduler,
                )

            return create(subscribe)

        # Allow using as a sink directly with subscribe()
        def on_next(self, value):
            self._sink.on_next(value)

        def on_error(self, error):
            self._sink.on_error(error)

        def on_completed(self):
            self._sink.on_completed()

    return NetworkOutputOperator(node)
