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

from datetime import datetime, timedelta
import cv2
import numpy as np
import os
from reactivex import Observable, Observer, create
from reactivex import operators as ops
from typing import Any, Callable, Tuple, Optional

import zmq
import base64
from enum import Enum

from dimos.stream.frame_processor import FrameProcessor

class VideoOperators:
    """Collection of video processing operators for reactive video streams."""
    
    @staticmethod
    def with_fps_sampling(
        fps: int = 25,
        *,
        sample_interval: Optional[timedelta] = None,
        use_latest: bool = True
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that samples frames at a specified rate.

        Creates a transformation operator that samples frames either by taking
        the latest frame or the first frame in each interval. Provides frame
        rate control through time-based selection.

        Args:
            fps: Desired frames per second, defaults to 25 FPS.
                Ignored if sample_interval is provided.
            sample_interval: Optional explicit interval between samples.
                If provided, overrides the fps parameter.
            use_latest: If True, uses the latest frame in interval.
                If False, uses the first frame. Defaults to True.

        Returns:
            A function that transforms an Observable[np.ndarray] stream to a sampled 
            Observable[np.ndarray] stream with controlled frame rate.

        Raises:
            ValueError: If fps is not positive or sample_interval is negative.
            TypeError: If sample_interval is provided but not a timedelta object.

        Examples:
            Sample latest frame at 30 FPS (good for real-time):
                >>> video_stream.pipe(
                ...     VideoOperators.with_fps_sampling(fps=30)
                ... )

            Sample first frame with custom interval (good for consistent timing):
                >>> video_stream.pipe(
                ...     VideoOperators.with_fps_sampling(
                ...         sample_interval=timedelta(milliseconds=40),
                ...         use_latest=False
                ...     )
                ... )

        Note:
            This operator helps manage high-speed video streams through time-based
            frame selection. It reduces the frame rate by selecting frames at 
            specified intervals.

            When use_latest=True:
                - Uses sampling to select the most recent frame at fixed intervals
                - Discards intermediate frames, keeping only the latest
                - Best for real-time video where latest frame is most relevant
                - Uses ops.sample internally

            When use_latest=False:
                - Uses throttling to select the first frame in each interval
                - Ignores subsequent frames until next interval
                - Best for scenarios where you want consistent frame timing
                - Uses ops.throttle_first internally

            This is an approropriate solution for managing video frame rates and
            memory usage in many scenarios.
        """
        if sample_interval is None:
            if fps <= 0:
                raise ValueError("FPS must be positive")
            sample_interval = timedelta(microseconds=int(1_000_000 / fps))

        def _operator(source: Observable) -> Observable:
            return source.pipe(
                ops.sample(sample_interval) if use_latest else ops.throttle_first(sample_interval)
            )
        return _operator

    @staticmethod
    def with_jpeg_export(
        frame_processor: 'FrameProcessor',
        save_limit: int = 100,
        suffix: str = "",
        loop: bool = False
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that saves video frames as JPEG files.

        Creates a transformation operator that saves each frame from the video
        stream as a JPEG file while passing the frame through unchanged.

        Args:
            frame_processor: FrameProcessor instance that handles the JPEG export
                operations and maintains file count.
            save_limit: Maximum number of frames to save before stopping.
                Defaults to 100. Set to 0 for unlimited saves.
            suffix: Optional string to append to filename before index.
                Example: "raw" creates "1_raw.jpg".
                Defaults to empty string.
            loop: If True, when save_limit is reached, the files saved are 
                loopbacked and overwritten with the most recent frame.
                Defaults to False.
        Returns:
            A function that transforms an Observable of frames into another
            Observable of the same frames, with side effect of saving JPEGs.

        Raises:
            ValueError: If save_limit is negative.
            TypeError: If frame_processor is not a FrameProcessor instance.

        Example:
            >>> video_stream.pipe(
            ...     VideoOperators.with_jpeg_export(processor, suffix="raw")
            ... )
        """
        def _operator(source: Observable) -> Observable:
            return source.pipe(
                ops.map(lambda frame: frame_processor.export_to_jpeg(frame, save_limit, loop, suffix))
            )
        return _operator
    
    @staticmethod
    def with_optical_flow_filtering(
        threshold: float = 1.0
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that filters optical flow frames by relevancy score.

        Filters a stream of optical flow results (frame, relevancy_score) tuples,
        passing through only frames that meet the relevancy threshold.

        Args:
            threshold: Minimum relevancy score required for frames to pass through.
                Defaults to 1.0. Higher values mean more motion required.

        Returns:
            A function that transforms an Observable of (frame, score) tuples
            into an Observable of frames that meet the threshold.

        Raises:
            ValueError: If threshold is negative.
            TypeError: If input stream items are not (frame, float) tuples.

        Examples:
            Basic filtering:
                >>> optical_flow_stream.pipe(
                ...     VideoOperators.with_optical_flow_filtering(threshold=1.0)
                ... )

            With custom threshold:
                >>> optical_flow_stream.pipe(
                ...     VideoOperators.with_optical_flow_filtering(threshold=2.5)
                ... )

        Note:
            Input stream should contain tuples of (frame, relevancy_score) where
            frame is a numpy array and relevancy_score is a float or None.
            None scores are filtered out.
        """
        return lambda source: source.pipe(
            ops.filter(lambda result: result[1] is not None),
            ops.filter(lambda result: result[1] > threshold),
            ops.map(lambda result: result[0])
        )

    @staticmethod
    def with_edge_detection(
        frame_processor: 'FrameProcessor',
    ) -> Callable[[Observable], Observable]:
        return lambda source: source.pipe(
            ops.map(lambda frame: frame_processor.edge_detection(frame))
        )

    @staticmethod
    def with_optical_flow(
        frame_processor: 'FrameProcessor',
    ) -> Callable[[Observable], Observable]:
        return lambda source: source.pipe(
            ops.scan(
                lambda acc, frame: frame_processor.compute_optical_flow(acc, frame, compute_relevancy=False),
                (None, None, None)
            ),
            ops.map(lambda result: result[1]),  # Extract flow component
            ops.filter(lambda flow: flow is not None),
            ops.map(frame_processor.visualize_flow),
        )

    @staticmethod
    def with_zmq_socket(
        socket: zmq.Socket,
        scheduler: Optional[Any] = None
    ) -> Callable[[Observable], Observable]:
        def send_frame(frame, socket):
            _, img_encoded = cv2.imencode('.jpg', frame)
            socket.send(img_encoded.tobytes())
            # print(f"Frame received: {frame.shape}")

        # Use a default scheduler if none is provided
        if scheduler is None:
            from reactivex.scheduler import ThreadPoolScheduler
            scheduler = ThreadPoolScheduler(1)  # Single-threaded pool for isolation

        return lambda source: source.pipe(
            ops.observe_on(scheduler),  # Ensure this part runs on its own thread
            ops.do_action(lambda frame: send_frame(frame, socket)),
        )

    @staticmethod
    def encode_image() -> Callable[[Observable], Observable]:
        """
        Operator to encode an image to JPEG format and convert it to a Base64 string.

        Returns:
            A function that transforms an Observable of images into an Observable
            of tuples containing the Base64 string of the encoded image and its dimensions.
        """
        def _operator(source: Observable) -> Observable:
            def _encode_image(image: np.ndarray) -> Tuple[str, Tuple[int, int]]:
                try:
                    width, height = image.shape[:2]
                    _, buffer = cv2.imencode('.jpg', image)
                    if buffer is None:
                        raise ValueError("Failed to encode image")
                    base64_image = base64.b64encode(buffer).decode('utf-8')
                    return base64_image, (width, height)
                except Exception as e:
                    raise e

            return source.pipe(
                ops.map(_encode_image)
            )

        return _operator

from reactivex.disposable import Disposable
from reactivex import Observable, create
from threading import Lock

class Operators:

    @staticmethod
    def exhaust_lock(process_item):
        """
        For each incoming item, call `process_item(item)` to get an Observable.
        - If we're busy processing the previous one, skip new items.
        - Use a lock to ensure concurrency safety across threads.
        """
        def _exhaust_lock(source: Observable) -> Observable:
            def _subscribe(observer, scheduler=None):
                in_flight = False
                lock = Lock()
                upstream_done = False

                upstream_disp = None
                active_inner_disp = None

                def dispose_all():
                    if upstream_disp:
                        upstream_disp.dispose()
                    if active_inner_disp:
                        active_inner_disp.dispose()

                def on_next(value):
                    nonlocal in_flight, active_inner_disp
                    lock.acquire()
                    try:
                        if not in_flight:
                            in_flight = True
                            print("Processing new item.")
                        else:
                            print("Skipping item, already processing.")
                            return
                    finally:
                        lock.release()

                    # We only get here if we grabbed the in_flight slot
                    try:
                        inner_source = process_item(value)
                    except Exception as ex:
                        observer.on_error(ex)
                        return

                    def inner_on_next(ivalue):
                        observer.on_next(ivalue)

                    def inner_on_error(err):
                        nonlocal in_flight
                        with lock:
                            in_flight = False
                        observer.on_error(err)

                    def inner_on_completed():
                        nonlocal in_flight
                        with lock:
                            in_flight = False
                            if upstream_done:
                                observer.on_completed()

                    # Subscribe to the inner observable
                    nonlocal active_inner_disp
                    active_inner_disp = inner_source.subscribe(
                        on_next=inner_on_next,
                        on_error=inner_on_error,
                        on_completed=inner_on_completed,
                        scheduler=scheduler
                    )

                def on_error(err):
                    dispose_all()
                    observer.on_error(err)

                def on_completed():
                    nonlocal upstream_done
                    with lock:
                        upstream_done = True
                        # If we're not busy, we can end now
                        if not in_flight:
                            observer.on_completed()

                upstream_disp = source.subscribe(
                    on_next,
                    on_error,
                    on_completed,
                    scheduler=scheduler
                )
                return dispose_all

            return create(_subscribe)
        return _exhaust_lock
    
    @staticmethod
    def exhaust_lock_per_instance(process_item, lock: Lock):
        """
        - For each item from upstream, call process_item(item) -> Observable.
        - If a frame arrives while one is "in flight", discard it.
        - 'lock' ensures we safely check/modify the 'in_flight' state in a multithreaded environment.
        """
        def _exhaust_lock(source: Observable) -> Observable:
            def _subscribe(observer, scheduler=None):
                in_flight = False
                upstream_done = False

                upstream_disp = None
                active_inner_disp = None

                def dispose_all():
                    if upstream_disp:
                        upstream_disp.dispose()
                    if active_inner_disp:
                        active_inner_disp.dispose()

                def on_next(value):
                    nonlocal in_flight, active_inner_disp
                    with lock:
                        # If not busy, claim the slot
                        if not in_flight:
                            in_flight = True
                            print("\033[34mProcessing new item.\033[0m")
                        else:
                            # Already processing => drop
                            print("\033[34mSkipping item, already processing.\033[0m")
                            return

                    # We only get here if we acquired the slot
                    try:
                        inner_source = process_item(value)
                    except Exception as ex:
                        observer.on_error(ex)
                        return

                    def inner_on_next(ivalue):
                        observer.on_next(ivalue)

                    def inner_on_error(err):
                        nonlocal in_flight
                        with lock:
                            in_flight = False
                            print("\033[34mError in inner on error.\033[0m")
                        observer.on_error(err)

                    def inner_on_completed():
                        nonlocal in_flight
                        with lock:
                            in_flight = False
                            print("\033[34mInner on completed.\033[0m")
                            if upstream_done:
                                observer.on_completed()

                    # Subscribe to the inner Observable
                    nonlocal active_inner_disp
                    active_inner_disp = inner_source.subscribe(
                        on_next=inner_on_next,
                        on_error=inner_on_error,
                        on_completed=inner_on_completed,
                        scheduler=scheduler
                    )

                def on_error(e):
                    dispose_all()
                    observer.on_error(e)

                def on_completed():
                    nonlocal upstream_done
                    with lock:
                        upstream_done = True
                        print("\033[34mOn completed.\033[0m")
                        if not in_flight:
                            observer.on_completed()

                upstream_disp = source.subscribe(
                    on_next=on_next,
                    on_error=on_error,
                    on_completed=on_completed,
                    scheduler=scheduler
                )

                return Disposable(dispose_all)

            return create(_subscribe)
        return _exhaust_lock

    @staticmethod
    def exhaust_map(project):
        def _exhaust_map(source: Observable):
            def subscribe(observer, scheduler=None):
                is_processing = False

                def on_next(item):
                    nonlocal is_processing
                    if not is_processing:
                        is_processing = True
                        print(f"\033[35mProcessing item.\033[0m")
                        try:
                            inner_observable = project(item)  # Create the inner observable
                            inner_observable.subscribe(
                                on_next=observer.on_next,
                                on_error=observer.on_error,
                                on_completed=lambda: set_not_processing(),
                                scheduler=scheduler
                            )
                        except Exception as e:
                            observer.on_error(e)
                    else:
                        print(f"\033[35mSkipping item, already processing.\033[0m")
                
                def set_not_processing():
                    nonlocal is_processing
                    is_processing = False
                    print(f"\033[35mItem processed.\033[0m")

                return source.subscribe(
                    on_next=on_next, 
                    on_error=observer.on_error, 
                    on_completed=observer.on_completed, 
                    scheduler=scheduler
                )

            return create(subscribe)

        return _exhaust_map
    
    @staticmethod
    def with_lock(lock: Lock):
        def operator(source: Observable):
            def subscribe(observer, scheduler=None):
                def on_next(item):
                    if not lock.locked():  # Check if the lock is free
                        if lock.acquire(blocking=False): # Non-blocking acquire
                            try:
                                print(f"\033[32mAcquired lock, processing item.\033[0m")
                                observer.on_next(item)
                            finally: # Ensure lock release even if observer.on_next throws
                                lock.release()
                        else:
                            print(f"\033[34mLock busy, skipping item.\033[0m")
                    else:
                        print(f"\033[34mLock busy, skipping item.\033[0m")

                def on_error(error):
                    observer.on_error(error)

                def on_completed():
                    observer.on_completed()

                return source.subscribe(
                    on_next=on_next, on_error=on_error, on_completed=on_completed, scheduler=scheduler
                )

            return Observable(subscribe)

        return operator
    
    @staticmethod
    def with_lock_check(lock: Lock):  # Renamed for clarity
        def operator(source: Observable):
            def subscribe(observer, scheduler=None):
                def on_next(item):
                    if not lock.locked():  # Check if the lock is held WITHOUT acquiring
                        print(f"\033[32mLock is free, processing item: {item}\033[0m")
                        observer.on_next(item)
                    else:
                        print(f"\033[34mLock is busy, skipping item: {item}\033[0m")
                        # observer.on_completed()

                def on_error(error):
                    observer.on_error(error)

                def on_completed():
                    observer.on_completed()

                return source.subscribe(
                    on_next=on_next, on_error=on_error, on_completed=on_completed, scheduler=scheduler
                )

            return Observable(subscribe)

        return operator

    # PrintColor enum for standardized color formatting
    class PrintColor(Enum):
        RED = "\033[31m"
        GREEN = "\033[32m"
        BLUE = "\033[34m"
        YELLOW = "\033[33m"
        MAGENTA = "\033[35m"
        CYAN = "\033[36m"
        WHITE = "\033[37m"
        RESET = "\033[0m"

    @staticmethod
    def print_emission(id: str, 
                       dev_name: str = "NA", 
                       counts: dict = None, 
                       color: 'Operators.PrintColor' = None, 
                       enabled: bool = True):
        """
        Creates an operator that prints the emission with optional counts for debugging.
        
        Args:
            id: Identifier for the emission point (e.g., 'A', 'B')
            dev_name: Device or component name for context
            counts: External dictionary to track emission count across operators. If None, will not print emission count.
            color: Color for the printed output from PrintColor enum (default is RED)
            enabled: Whether to print the emission count (default is True)
        Returns:
            An operator that counts and prints emissions without modifying the stream
        """
        # If enabled is false, return the source unchanged
        if not enabled:
            return lambda source: source
        
        # Use RED as default if no color provided
        if color is None:
            color = Operators.PrintColor.RED

        def _operator(source: Observable) -> Observable:
            def _subscribe(observer: Observer, scheduler=None):
                def on_next(value):
                    if counts is not None:
                        # Initialize count if necessary
                        if id not in counts:
                            counts[id] = 0
                    
                        # Increment and print
                        counts[id] += 1
                        print(f"{color.value}({dev_name} - {id}) Emission Count - {counts[id]} {datetime.now()}{Operators.PrintColor.RESET.value}")
                    else: 
                        print(f"{color.value}({dev_name} - {id}) Emitted - {datetime.now()}{Operators.PrintColor.RESET.value}")

                    # Pass value through unchanged
                    observer.on_next(value)
                    
                return source.subscribe(
                    on_next=on_next,
                    on_error=observer.on_error,
                    on_completed=observer.on_completed,
                    scheduler=scheduler
                )
            
            return create(_subscribe)
        
        return _operator
    