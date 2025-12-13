#!/usr/bin/env python3
"""
Shared memory transport for high-performance image data transfer.
Drop-in replacement for LCMTransport to avoid serialization overhead.
"""

import numpy as np
import time
import threading
from typing import Optional, Tuple, Callable
from dataclasses import dataclass
from multiprocessing import shared_memory

@dataclass
class SharedMemoryImage:
    """Shared memory image with metadata"""
    data: np.ndarray
    timestamp: float
    sequence: int
    frame_id: str

class SharedMemoryTransport:
    """High-performance shared memory transport for image data"""
    
    def __init__(self, topic_name: str, shape: Tuple[int, ...], dtype: np.dtype, buffer_count: int = 2):
        self.topic_name = topic_name
        self.shape = shape
        self.dtype = np.dtype(dtype)
        self.buffer_count = buffer_count
        
        # Calculate buffer size
        self.buffer_size = int(np.prod(shape) * self.dtype.itemsize)
        self.total_size = self.buffer_size * buffer_count + 1024  # Extra space for metadata
        
        # Create shared memory name from topic
        self.shm_name = f"dimos_{topic_name.replace('/', '_')}"
        
        # Try to create new shared memory, otherwise attach
        try:
            self.shm = shared_memory.SharedMemory(name=self.shm_name, create=True, size=self.total_size)
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name=self.shm_name, create=False, size=self.total_size)
        
        self._attach_buffers()
        
        self.lock = threading.Lock()
        self.subscribers = []
    
    def _attach_buffers(self):
        """(Re)attach numpy views to shared memory."""
        self.buffers = []
        for i in range(self.buffer_count):
            offset = i * self.buffer_size
            buffer_array = np.ndarray(
                self.shape,
                dtype=self.dtype,
                buffer=self.shm.buf,
                offset=offset
            )
            self.buffers.append(buffer_array)
        
        # Metadata region (last 1024 bytes)
        self.metadata_offset = self.buffer_size * self.buffer_count
        self.metadata = np.ndarray(
            (128,),  # 1024 / 8 bytes per uint64
            dtype=np.uint64,
            buffer=self.shm.buf,
            offset=self.metadata_offset
        )
        
        # Initialize metadata if fresh
        if self.metadata[0] == 0 and self.metadata[1] == 0:
            self.metadata[0] = 0  # Current buffer index
            self.metadata[1] = 0  # Sequence number
            self.metadata[2] = 0  # Timestamp
            self.metadata[3] = 0  # Frame ID hash
    
    def publish(self, image_msg):
        if hasattr(image_msg, 'data'):
            data = image_msg.data
        else:
            data = image_msg
        
        frame_id = getattr(image_msg, 'frame_id', 'camera_link')
        timestamp = getattr(image_msg, 'ts', time.time())
        
        if data.shape != self.shape or data.dtype != self.dtype:
            raise ValueError(f"Data shape {data.shape} or dtype {data.dtype} doesn't match expected {self.shape} {self.dtype}")
        
        with self.lock:
            current_idx = int(self.metadata[0])
            next_idx = (current_idx + 1) % self.buffer_count
            np.copyto(self.buffers[next_idx], data)
            
            self.metadata[0] = next_idx
            self.metadata[1] += 1
            self.metadata[2] = int(timestamp * 1e9)
            self.metadata[3] = hash(frame_id)
        
        for subscriber in self.subscribers:
            try:
                subscriber(image_msg)
            except Exception as e:
                print(f"Error in subscriber: {e}")
    
    def subscribe(self, callback: Callable, selfstream=None):
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback) if callback in self.subscribers else None
    
    def get_latest(self) -> Optional[SharedMemoryImage]:
        current_idx = int(self.metadata[0])
        sequence = int(self.metadata[1])
        timestamp_ns = int(self.metadata[2])
        frame_id_hash = int(self.metadata[3])
        
        data_copy = self.buffers[current_idx].copy()
        
        return SharedMemoryImage(
            data=data_copy,
            timestamp=timestamp_ns / 1e9,
            sequence=sequence,
            frame_id=f"frame_{frame_id_hash}"
        )
    
    def close(self):
        if hasattr(self, 'shm'):
            self.shm.close()
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass

    # --- Pickle support ---
    def __getstate__(self):
        # Only store minimal info required to reattach
        return {
            "topic_name": self.topic_name,
            "shape": self.shape,
            "dtype": self.dtype.str,   # store dtype as string
            "buffer_count": self.buffer_count,
        }

    def __setstate__(self, state):
        self.topic_name = state["topic_name"]
        self.shape = state["shape"]
        self.dtype = np.dtype(state["dtype"])
        self.buffer_count = state["buffer_count"]

        self.buffer_size = int(np.prod(self.shape) * self.dtype.itemsize)
        self.total_size = self.buffer_size * self.buffer_count + 1024
        self.shm_name = f"dimos_{self.topic_name.replace('/', '_')}"

        # Reattach to existing shm
        self.shm = shared_memory.SharedMemory(name=self.shm_name, create=False, size=self.total_size)
        self._attach_buffers()
        
        # Recreate transient members
        self.lock = threading.Lock()
        self.subscribers = []

