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

import asyncio
import logging
import threading

# this is missing, I'm just trying to import lcm_foxglove_bridge.py from dimos_lcm
from dimos_lcm.foxglove_bridge import (  # type: ignore[import-untyped]
    FoxgloveBridge as LCMFoxgloveBridge,
)

from dimos.core import DimosCluster, Module, rpc

logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)


class FoxgloveBridge(Module):
    _thread: threading.Thread | None = None
    _loop: asyncio.AbstractEventLoop | None = None

    def __init__(self, *args, shm_channels=None, jpeg_shm_channels=None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        # IMPORTANT: We accept jpeg_shm_channels here, but we will NEVER pass it to LCMFoxgloveBridge
        # Store parameters
        self.shm_channels = shm_channels or []
        self.jpeg_shm_channels = jpeg_shm_channels or []
        
        # Print immediately to stderr so we see it even if logging is suppressed
        import sys
        print(f"[FoxgloveBridge.__init__] Called with shm_channels={shm_channels}, jpeg_shm_channels={jpeg_shm_channels}", file=sys.stderr, flush=True)
        print(f"[FoxgloveBridge.__init__] kwargs keys: {list(kwargs.keys())}", file=sys.stderr, flush=True)
        
        # Log initialization
        logger = logging.getLogger(__name__)
        logger.info(f"[FoxgloveBridge.__init__] Initializing with shm_channels={self.shm_channels}, jpeg_shm_channels={self.jpeg_shm_channels}")
        
        super().__init__(*args, **kwargs)
        print("[FoxgloveBridge.__init__] Initialized successfully", file=sys.stderr, flush=True)
        logger.info("[FoxgloveBridge.__init__] Initialized successfully")

    @rpc
    def start(self) -> None:
        super().start()

        def run_bridge() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            try:
                for logger in ["lcm_foxglove_bridge", "FoxgloveServer"]:
                    logger = logging.getLogger(logger)  # type: ignore[assignment]
                    logger.setLevel(logging.ERROR)  # type: ignore[attr-defined]
                    for handler in logger.handlers:  # type: ignore[attr-defined]
                        handler.setLevel(logging.ERROR)

                logger = logging.getLogger(__name__)
                logger.info("[FoxgloveBridge] Starting bridge on port 8765...")
                logger.info(f"[FoxgloveBridge] shm_channels: {self.shm_channels}")
                if self.jpeg_shm_channels:
                    logger.warning(f"[FoxgloveBridge] jpeg_shm_channels not supported by LCMFoxgloveBridge, ignoring: {self.jpeg_shm_channels}")
                
                # Build kwargs for LCMFoxgloveBridge
                # LCMFoxgloveBridge accepts: host, port, debug, num_threads, shm_channels
                # It does NOT accept jpeg_shm_channels - we explicitly exclude it
                bridge_kwargs: dict[str, any] = {
                    "host": "0.0.0.0",
                    "port": 8765,
                    "debug": False,
                    "num_threads": 4,
                }
                
                # Add shm_channels if provided (and not empty)
                if self.shm_channels:
                    bridge_kwargs["shm_channels"] = self.shm_channels
                
                # CRITICAL: Ensure jpeg_shm_channels is NEVER in kwargs
                # This should never happen, but we're being extra defensive
                if "jpeg_shm_channels" in bridge_kwargs:
                    logger.error("[FoxgloveBridge] CRITICAL: jpeg_shm_channels found in bridge_kwargs! Removing it...")
                    del bridge_kwargs["jpeg_shm_channels"]
                
                logger.info(f"[FoxgloveBridge] Creating bridge with kwargs: {list(bridge_kwargs.keys())}")
                logger.info(f"[FoxgloveBridge] Full bridge_kwargs: {bridge_kwargs}")
                
                # Create the bridge - this is where the error would occur if jpeg_shm_channels was passed
                # Wrap in try-except to catch and display the exact error
                try:
                    logger.info("[FoxgloveBridge] About to instantiate LCMFoxgloveBridge...")
                    bridge = LCMFoxgloveBridge(**bridge_kwargs)
                    logger.info("[FoxgloveBridge] Bridge initialized successfully, starting server...")
                    logger.info("[FoxgloveBridge] Bridge will automatically forward all LCM topics to Foxglove")
                    self._loop.run_until_complete(bridge.run())
                except TypeError as e:
                    # This is the specific error we're seeing
                    import sys
                    import traceback
                    error_details = f"""
[FoxgloveBridge] TYPEERROR when creating LCMFoxgloveBridge:
  Error: {e}
  Bridge kwargs passed: {bridge_kwargs}
  Bridge kwargs keys: {list(bridge_kwargs.keys())}
  jpeg_shm_channels in kwargs: {'jpeg_shm_channels' in bridge_kwargs}
  Full traceback:
{traceback.format_exc()}
"""
                    logger.error(error_details)
                    print(error_details, file=sys.stderr, flush=True)
                    raise  # Re-raise so it gets caught by outer exception handler
            except Exception as e:
                import traceback
                import sys
                logger = logging.getLogger(__name__)
                full_traceback = traceback.format_exc()
                error_msg = f"[FoxgloveBridge] ERROR in bridge thread: {e}\n{full_traceback}"
                
                # Log with full traceback
                logger.error(error_msg, exc_info=True)
                
                # Also print to stderr for visibility (this is what the user sees)
                print(f"Foxglove bridge error: {e}", file=sys.stderr, flush=True)
                print("Full traceback:", file=sys.stderr, flush=True)
                print(full_traceback, file=sys.stderr, flush=True)
                
                # Don't re-raise - let the thread die gracefully so the rest of the system continues

        self._thread = threading.Thread(target=run_bridge, daemon=True, name="FoxgloveBridge")
        self._thread.start()
        import sys
        print(f"[FoxgloveBridge] Bridge thread started (daemon={self._thread.daemon})", file=sys.stderr, flush=True)

    @rpc
    def stop(self) -> None:
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2)

        super().stop()


def deploy(
    dimos: DimosCluster,
    shm_channels: list[str] | None = None,
) -> FoxgloveBridge:
    if shm_channels is None:
        shm_channels = [
            "/image#sensor_msgs.Image",
            "/lidar#sensor_msgs.PointCloud2",
            "/map#sensor_msgs.PointCloud2",
        ]
    foxglove_bridge = dimos.deploy(  # type: ignore[attr-defined]
        FoxgloveBridge,
        shm_channels=shm_channels,
    )
    foxglove_bridge.start()
    return foxglove_bridge  # type: ignore[no-any-return]


foxglove_bridge = FoxgloveBridge.blueprint


__all__ = ["FoxgloveBridge", "deploy", "foxglove_bridge"]
