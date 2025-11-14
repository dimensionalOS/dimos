"""
Queue-based command management system for robot commands.

This module provides a unified approach to queueing and processing all robot commands,
including WebRTC requests and action client commands.
Commands are processed sequentially and only when the robot is in IDLE state.
"""

import threading
import time
import uuid
from enum import Enum, auto
from queue import PriorityQueue, Empty
from typing import Callable, Optional, NamedTuple, Dict, Any, Tuple, List
from dimos.utils.logging_config import setup_logger
import logging

# Initialize logger for the ros command queue module
logger = setup_logger("dimos.robot.ros_command_queue", level=logging.DEBUG)

class CommandType(Enum):
    """Types of commands that can be queued"""
    WEBRTC = auto()  # WebRTC API requests
    ACTION = auto()   # Any action client or function call

class WebRTCRequest(NamedTuple):
    """Class to represent a WebRTC request in the queue"""
    id: str  # Unique ID for tracking
    api_id: int  # API ID for the command
    topic: str  # Topic to publish to
    parameter: str  # Optional parameter string
    priority: int  # Priority level
    timeout: float  # How long to wait for this request to complete

class ROSCommand(NamedTuple):
    """Class to represent a command in the queue"""
    id: str  # Unique ID for tracking
    cmd_type: CommandType  # Type of command
    execute_func: Callable  # Function to execute the command
    params: Dict[str, Any]  # Parameters for the command (for debugging/logging)
    priority: int  # Priority level (lower is higher priority)
    timeout: float  # How long to wait for this command to complete

class ROSCommandQueue:
    """
    Manages a queue of commands for the robot.
    
    Commands are executed sequentially, with only one command being processed at a time.
    Commands are only executed when the robot is in the IDLE state.
    """
    
    def __init__(self, 
                 webrtc_func: Callable,
                 is_ready_func: Callable[[], bool] = None,
                 is_busy_func: Optional[Callable[[], bool]] = None,
                 debug: bool = True):
        """
        Initialize the ROSCommandQueue.
        
        Args:
            webrtc_func: Function to send WebRTC requests
            is_ready_func: Function to check if the robot is ready for a command
            is_busy_func: Function to check if the robot is busy
            debug: Whether to enable debug logging
        """
        self._webrtc_func = webrtc_func
        self._is_ready_func = is_ready_func or (lambda: True)
        self._is_busy_func = is_busy_func
        self._debug = debug
        
        # Queue of commands to process
        self._queue = PriorityQueue()
        self._current_command = None
        self._last_command_time = 0
        
        # Last known robot state
        self._last_ready_state = None
        self._last_busy_state = None
        self._stuck_in_busy_since = None
        
        # Command execution status
        self._should_stop = False
        self._queue_thread = None
        
        # Stats
        self._command_count = 0
        self._success_count = 0
        self._failure_count = 0
        self._command_history = []
        
        self._max_queue_wait_time = 30.0  # Maximum time to wait for robot to be ready before forcing
        
        logger.info("ROSCommandQueue initialized")
        
    def start(self):
        """Start the queue processing thread"""
        if self._queue_thread is not None and self._queue_thread.is_alive():
            logger.warning("Queue processing thread already running")
            return
            
        self._should_stop = False
        self._queue_thread = threading.Thread(target=self._process_queue, daemon=True)
        self._queue_thread.start()
        logger.info("Queue processing thread started")
        
    def stop(self, timeout=2.0):
        """
        Stop the queue processing thread
        
        Args:
            timeout: Maximum time to wait for the thread to stop
        """
        if self._queue_thread is None or not self._queue_thread.is_alive():
            logger.warning("Queue processing thread not running")
            return
            
        self._should_stop = True
        try:
            self._queue_thread.join(timeout=timeout)
            if self._queue_thread.is_alive():
                logger.warning(f"Queue processing thread did not stop within {timeout}s")
            else:
                logger.info("Queue processing thread stopped")
        except Exception as e:
            logger.error(f"Error stopping queue processing thread: {e}")
        
    def queue_webrtc_request(self, api_id: int, topic: str = None, parameter: str = '', 
                             request_id: str = None, data: Dict[str, Any] = None,
                             priority: int = 0, timeout: float = 30.0) -> str:
        """
        Queue a WebRTC request
        
        Args:
            api_id: API ID for the command
            topic: Topic to publish to
            parameter: Optional parameter string
            request_id: Unique ID for the request (will be generated if not provided)
            data: Data to include in the request
            priority: Priority level (lower is higher priority)
            timeout: Maximum time to wait for the command to complete
            
        Returns:
            str: Unique ID for the request
        """
        request_id = request_id or str(uuid.uuid4())
        
        # Create a function that will execute this WebRTC request
        def execute_webrtc():
            try:
                logger.info(f"Executing WebRTC request: {api_id} (ID: {request_id})")
                if self._debug:
                    logger.debug(f"[WebRTC Queue] SENDING request: API ID {api_id}")
                
                result = self._webrtc_func(
                    api_id=api_id, 
                    topic=topic,
                    parameter=parameter,
                    request_id=request_id,
                    data=data,
                )
                if not result:
                    logger.warning(f"WebRTC request failed: {api_id} (ID: {request_id})")
                    if self._debug:
                        logger.debug(f"[WebRTC Queue] Request API ID {api_id} FAILED to send")
                    return False
                
                if self._debug:
                    logger.debug(f"[WebRTC Queue] Request API ID {api_id} sent SUCCESSFULLY")
                
                # Allow time for the robot to process the command
                start_time = time.time()
                stabilization_delay = 0.5  # Half-second delay for stabilization
                time.sleep(stabilization_delay)
                
                # Wait for the robot to complete the command (timeout check)
                while self._is_busy_func() and (time.time() - start_time) < timeout:
                    if self._debug and (time.time() - start_time) % 5 < 0.1:  # Print every ~5 seconds
                        logger.debug(f"[WebRTC Queue] Still waiting on API ID {api_id} - elapsed: {time.time()-start_time:.1f}s")
                    time.sleep(0.1)
                
                # Check if we timed out
                if self._is_busy_func() and (time.time() - start_time) >= timeout:
                    logger.warning(f"WebRTC request timed out: {api_id} (ID: {request_id})")
                    return False
                
                wait_time = time.time() - start_time
                if self._debug:
                    logger.debug(f"[WebRTC Queue] Request API ID {api_id} completed after {wait_time:.1f}s")
                
                logger.info(f"WebRTC request completed: {api_id} (ID: {request_id})")
                return True
            except Exception as e:
                logger.error(f"Error executing WebRTC request: {e}")
                if self._debug:
                    logger.debug(f"[WebRTC Queue] ERROR processing request: {e}")
                return False
        
        # Create the command and queue it
        command = ROSCommand(
            id=request_id,
            cmd_type=CommandType.WEBRTC,
            execute_func=execute_webrtc,
            params={'api_id': api_id, 'topic': topic, 'request_id': request_id},
            priority=priority,
            timeout=timeout
        )
        
        # Queue the command
        self._queue.put((priority, self._command_count, command))
        self._command_count += 1
        if self._debug:
            logger.debug(f"[WebRTC Queue] Added request ID {request_id} for API ID {api_id} - Queue size now: {self.queue_size}")
        logger.info(f"Queued WebRTC request: {api_id} (ID: {request_id}, Priority: {priority})")
        
        return request_id
        
    def queue_action_client_request(self, action_name: str, execute_func: Callable,
                               priority: int = 0, timeout: float = 30.0, **kwargs) -> str:
        """
        Queue any action client request or function
        
        Args:
            action_name: Name of the action for logging/tracking
            execute_func: Function to execute the command
            priority: Priority level (lower is higher priority)
            timeout: Maximum time to wait for the command to complete
            **kwargs: Additional parameters to pass to the execute function
            
        Returns:
            str: Unique ID for the request
        """
        request_id = str(uuid.uuid4())
        
        # Create the command
        command = ROSCommand(
            id=request_id,
            cmd_type=CommandType.ACTION,
            execute_func=execute_func,
            params={'action_name': action_name, **kwargs},
            priority=priority,
            timeout=timeout
        )
        
        # Queue the command
        self._queue.put((priority, self._command_count, command))
        self._command_count += 1
        
        action_params = ', '.join([f"{k}={v}" for k, v in kwargs.items()])
        logger.info(f"Queued action request: {action_name} (ID: {request_id}, Priority: {priority}, Params: {action_params})")
        
        return request_id
        
    def _process_queue(self):
        """Process commands in the queue"""
        logger.info("Starting queue processing")
        logger.info("[WebRTC Queue] Processing thread started")
        
        while not self._should_stop:
            # Print queue status
            self._print_queue_status()
            
            # Check if we're ready to process a command
            if not self._queue.empty() and self._current_command is None:
                current_time = time.time()
                is_ready = self._is_ready_func()
                is_busy = self._is_busy_func() if self._is_busy_func else False
                
                if self._debug:
                    logger.debug(f"[WebRTC Queue] Status: {self.queue_size} requests waiting | Robot ready: {is_ready} | Robot busy: {is_busy}")
                
                # Track robot state changes
                if is_ready != self._last_ready_state:
                    logger.debug(f"Robot ready state changed: {self._last_ready_state} -> {is_ready}")
                    self._last_ready_state = is_ready
                    
                if is_busy != self._last_busy_state:
                    logger.debug(f"Robot busy state changed: {self._last_busy_state} -> {is_busy}")
                    self._last_busy_state = is_busy
                    
                    # If the robot has transitioned to busy, record the time
                    if is_busy:
                        self._stuck_in_busy_since = current_time
                    else:
                        self._stuck_in_busy_since = None
                
                # Check if we've been waiting too long for the robot to be ready
                force_processing = False
                if (not is_ready and is_busy and 
                    self._stuck_in_busy_since is not None and 
                    current_time - self._stuck_in_busy_since > self._max_queue_wait_time):
                    logger.warning(
                        f"Robot has been busy for {current_time - self._stuck_in_busy_since:.1f}s, "
                        f"forcing queue to continue"
                    )
                    force_processing = True
                
                # Process the next command if ready or forcing
                if is_ready or force_processing:
                    if self._debug and is_ready:
                        logger.debug("[WebRTC Queue] Robot is READY for next command")
                    
                    try:
                        # Get the next command
                        _, _, command = self._queue.get(block=False)
                        self._current_command = command
                        self._last_command_time = current_time
                        
                        # Log the command
                        cmd_info = f"ID: {command.id}, Type: {command.cmd_type.name}"
                        if command.cmd_type == CommandType.WEBRTC:
                            api_id = command.params.get('api_id')
                            cmd_info += f", API: {api_id}"
                            if self._debug:
                                logger.debug(f"[WebRTC Queue] DEQUEUED request: API ID {api_id}")
                        elif command.cmd_type == CommandType.ACTION:
                            action_name = command.params.get('action_name')
                            cmd_info += f", Action: {action_name}"
                            if self._debug:
                                logger.debug(f"[WebRTC Queue] DEQUEUED action: {action_name}")
                        
                        forcing_str = " (FORCED)" if force_processing else ""
                        logger.info(f"Processing command{forcing_str}: {cmd_info}")
                        
                        # Execute the command
                        try:
                            # Where command execution occurs
                            success = command.execute_func()
                            
                            if success:
                                self._success_count += 1
                                logger.info(f"Command succeeded: {cmd_info}")
                                if self._debug:
                                    logger.debug(f"[WebRTC Queue] Command {command.id} marked as COMPLETED")
                            else:
                                self._failure_count += 1
                                logger.warning(f"Command failed: {cmd_info}")
                                if self._debug:
                                    logger.debug(f"[WebRTC Queue] Command {command.id} FAILED")
                                
                            # Record command history
                            self._command_history.append({
                                'id': command.id,
                                'type': command.cmd_type.name,
                                'params': command.params,
                                'success': success,
                                'time': time.time() - self._last_command_time
                            })
                            
                        except Exception as e:
                            self._failure_count += 1
                            logger.error(f"Error executing command: {e}")
                            if self._debug:
                                logger.debug(f"[WebRTC Queue] ERROR executing command: {e}")
                        
                        # Mark the command as complete
                        self._current_command = None
                        if self._debug:
                            logger.debug("[WebRTC Queue] Adding 0.5s stabilization delay before next command")
                            time.sleep(0.5)
                            
                    except Empty:
                        pass
            
            # Sleep to avoid busy-waiting
            time.sleep(0.1)
        
        logger.info("Queue processing stopped")
        
    def _print_queue_status(self):
        """Print the current queue status"""
        current_time = time.time()
        
        # Only print once per second to avoid spamming the log
        if current_time - self._last_command_time < 1.0 and self._current_command is None:
            return
            
        is_ready = self._is_ready_func()
        is_busy = self._is_busy_func() if self._is_busy_func else False
        queue_size = self.queue_size
        
        # Get information about the current command
        current_command_info = "None"
        if self._current_command is not None:
            current_command_info = f"{self._current_command.cmd_type.name}"
            if self._current_command.cmd_type == CommandType.WEBRTC:
                api_id = self._current_command.params.get('api_id')
                current_command_info += f" (API: {api_id})"
            elif self._current_command.cmd_type == CommandType.ACTION:
                action_name = self._current_command.params.get('action_name')
                current_command_info += f" (Action: {action_name})"
            
        # Print the status
        status = (
            f"Queue: {queue_size} items | "
            f"Robot: {'READY' if is_ready else 'BUSY'} | "
            f"Current: {current_command_info} | "
            f"Stats: {self._success_count} OK, {self._failure_count} FAIL"
        )
        
        logger.debug(status)
        self._last_command_time = current_time
        
    @property
    def queue_size(self) -> int:
        """Get the number of commands in the queue"""
        return self._queue.qsize()
        
    @property
    def current_command(self) -> Optional[ROSCommand]:
        """Get the current command being processed"""
        return self._current_command
        