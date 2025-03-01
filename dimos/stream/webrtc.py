"""
WebRTC request handling and queue management.

This module provides utilities for managing WebRTC requests, particularly
for robot communication where requests need to be sent when the robot is
in specific states.
"""

import threading
import time
import uuid
from queue import Queue
from typing import Any, Callable, NamedTuple, Optional
from enum import Enum
import logging

logger = logging.getLogger(__name__)

class WebRTCRequest(NamedTuple):
    """Class to represent a WebRTC request in the queue"""
    id: str  # Unique ID for tracking
    api_id: int  # API ID for the command
    topic: str  # Topic to publish to
    parameter: str  # Optional parameter string
    priority: int  # Priority level
    timeout: float  # How long to wait for this request to complete

class WebRTCQueueManager:
    """
    Manages a queue of WebRTC requests to be sent when conditions are met.
    
    This class handles the queueing, processing, and sending of WebRTC requests
    in a background thread. It can be configured to only send requests when
    certain conditions are met (e.g., the robot is in IDLE state).
    """
    
    def __init__(self, 
                 send_request_func: Callable[[int, str, str, int], bool],
                 is_ready_func: Callable[[], bool],
                 is_busy_func: Optional[Callable[[], bool]] = None,
                 logger=None,
                 debug: bool = True):
        """
        Initialize the WebRTC queue manager.
        
        Args:
            send_request_func: Function to call to send a WebRTC request
            is_ready_func: Function that returns True when new requests can be processed
            is_busy_func: Function that returns True while a request is being processed
            logger: Logger instance to use (optional)
        """
        self._logger = logger or logging.getLogger(__name__)
        self._debug = debug
        # Store callback functions
        self._send_request = send_request_func
        self._is_ready = is_ready_func
        self._is_busy = is_busy_func or (lambda: False)
        
        # Initialize queue and state variables
        self._webrtc_queue = Queue()
        self._current_webrtc_request = None
        self._webrtc_lock = threading.Lock()
        self._should_stop = threading.Event()
        self._processing_thread = None
    
    def start(self):
        """Start the background processing thread if not already running."""
        if self._processing_thread is None or not self._processing_thread.is_alive():
            self._should_stop.clear()
            self._processing_thread = threading.Thread(
                target=self._process_queue,
                daemon=True
            )
            self._processing_thread.start()
            self._logger.info("WebRTC request queue processing thread started")

    def stop(self, timeout=2.0):
        """
        Stop the background processing thread.
        
        Args:
            timeout: Maximum time to wait for thread to stop gracefully
            
        Returns:
            bool: True if thread was stopped gracefully, False otherwise
        """
        if self._processing_thread and self._processing_thread.is_alive():
            self._logger.info("Stopping WebRTC queue processing thread...")
            self._should_stop.set()
            self._processing_thread.join(timeout=timeout)
            if self._processing_thread.is_alive():
                self._logger.warning("WebRTC processing thread did not stop gracefully")
                return False
            return True
        return True
    
    def queue_request(self, api_id: int, topic: str = 'rt/api/sport/request', 
                      parameter: str = '', priority: int = 0, 
                      timeout: float = 30.0) -> str:
        """
        Queue a WebRTC request to be sent when conditions are met.
        
        Args:
            api_id: The API ID for the command
            topic: The topic to publish to (e.g. 'rt/api/sport/request')
            parameter: Optional parameter string
            priority: Priority level (0 or 1)
            timeout: Maximum time to wait for the request to complete
            
        Returns:
            str: Request ID that can be used to track the request
        """
        request_id = str(uuid.uuid4())
        if self._debug:
            print(f"[WebRTC Queue] QUEUEING new request - API ID: {api_id}, Timeout: {timeout}s")
        
        # Create the request
        request = WebRTCRequest(
            id=request_id,
            api_id=api_id,
            topic=topic,
            parameter=parameter,
            priority=priority,
            timeout=timeout
        )
        
        # Add to queue
        with self._webrtc_lock:
            self._webrtc_queue.put(request)
            queue_size = self._webrtc_queue.qsize()

        if self._debug:
            print(f"[WebRTC Queue] Added request ID {request_id} for API ID {api_id} - Queue size now: {queue_size}")
        self._logger.info(f"Queued WebRTC request {request_id} (API ID: {api_id}) - Queue size: {queue_size}")
        
        # Start the processing thread if not already running
        self.start()
            
        return request_id
    
    def _process_queue(self):
        """
        Background thread for processing WebRTC requests when conditions are met.
        """
        self._logger.info("Started WebRTC request queue processing thread")
        print("[WebRTC Queue] Processing thread started")
        
        while not self._should_stop.is_set():
            # Only process requests when ready (e.g., robot is in IDLE mode)
            is_ready = self._is_ready()
            is_busy = self._is_busy()
            queue_size = self.queue_size
            if self._debug:
                print(f"[WebRTC Queue] Status: {queue_size} requests waiting | Robot ready: {is_ready} | Robot busy: {is_busy}")
            
            if is_ready:
                print("[WebRTC Queue] Robot is READY for next command")
                time.sleep(0.5)
                
                try:
                    # Get the next request from the queue (non-blocking)
                    with self._webrtc_lock:
                        if self._current_webrtc_request is None and not self._webrtc_queue.empty():
                            self._current_webrtc_request = self._webrtc_queue.get(block=False)
                            print(f"[WebRTC Queue] DEQUEUED request: API ID {self._current_webrtc_request.api_id}")
                            self._logger.info(f"Processing WebRTC request: {self._current_webrtc_request.id} (API ID: {self._current_webrtc_request.api_id})")
                    
                    # Process the current request if we have one
                    if self._current_webrtc_request:
                        req = self._current_webrtc_request
                        print(f"[WebRTC Queue] SENDING request: API ID {req.api_id}")
                        
                        # Send the request using the provided function
                        result = self._send_request(
                            api_id=req.api_id,
                            topic=req.topic,
                            parameter=req.parameter,
                            priority=req.priority
                        )
                        
                        if result:
                            print(f"[WebRTC Queue] Request API ID {req.api_id} sent SUCCESSFULLY")
                            self._logger.info(f"WebRTC request {req.id} sent successfully")
                        else:
                            print(f"[WebRTC Queue] Request API ID {req.api_id} FAILED to send")
                            self._logger.error(f"Failed to send WebRTC request {req.id}")
                        
                        # Wait for the request timeout or until no longer busy
                        start_time = time.time()
                        print(f"[WebRTC Queue] Waiting for request API ID {req.api_id} to complete (timeout: {req.timeout}s)")
                        
                        while (time.time() - start_time < req.timeout and 
                               self._is_busy() and
                               not self._should_stop.is_set()):
                            if (time.time() - start_time) % 5 < 0.1:  # Print every ~5 seconds
                                print(f"[WebRTC Queue] Still waiting on API ID {req.api_id} - elapsed: {time.time()-start_time:.1f}s")
                            time.sleep(0.1)
                        
                        wait_time = time.time() - start_time
                        print(f"[WebRTC Queue] Request API ID {req.api_id} completed after {wait_time:.1f}s")
                        
                        # Request is completed, clear it
                        with self._webrtc_lock:
                            self._current_webrtc_request = None
                            self._webrtc_queue.task_done()
                            print(f"[WebRTC Queue] Request API ID {req.api_id} marked as COMPLETED")
                            
                        # Add a small delay after each command completes
                        # This ensures the robot is fully stabilized before processing the next command
                        print("[WebRTC Queue] Adding 0.5s stabilization delay before next command")
                        time.sleep(0.5)
                            
                except Exception as e:
                    self._logger.error(f"Error processing WebRTC request: {e}")
                    print(f"[WebRTC Queue] ERROR processing request: {e}")
            
            # Sleep briefly to avoid CPU spinning
            time.sleep(0.1)
            
        self._logger.info("WebRTC request queue processing thread stopped")
        print("[WebRTC Queue] Processing thread stopped")
    
    @property
    def queue_size(self) -> int:
        """Get the current number of requests in the queue."""
        return self._webrtc_queue.qsize()
    
    @property
    def current_request(self) -> Optional[WebRTCRequest]:
        """Get the currently processing request (if any)."""
        return self._current_webrtc_request 