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

import threading
import time
from functools import wraps
from typing import Callable, Optional

from .accumulators import Accumulator, LatestAccumulator


def limit(max_freq: float, accumulator: Optional[Accumulator] = None):
    """
    Decorator that limits function call frequency.

    If calls come faster than max_freq, they are skipped.
    If calls come slower than max_freq, they pass through immediately.

    Args:
        max_freq: Maximum frequency in Hz (calls per second)
        accumulator: Optional accumulator to collect skipped calls (defaults to LatestAccumulator)

    Returns:
        Decorated function that respects the frequency limit
    """
    if max_freq <= 0:
        raise ValueError("Frequency must be positive")

    min_interval = 1.0 / max_freq

    # Create default accumulator if none provided
    if accumulator is None:
        accumulator = LatestAccumulator()

    def decorator(func: Callable) -> Callable:
        last_call_time = 0.0
        lock = threading.Lock()
        timer: Optional[threading.Timer] = None

        def execute_accumulated():
            nonlocal last_call_time, timer
            with lock:
                if len(accumulator):
                    acc_args, acc_kwargs = accumulator.get()
                    last_call_time = time.time()
                    timer = None
                    func(*acc_args, **acc_kwargs)

        @wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal last_call_time, timer
            current_time = time.time()

            with lock:
                time_since_last = current_time - last_call_time

                if time_since_last >= min_interval:
                    # Cancel any pending timer
                    if timer is not None:
                        timer.cancel()
                        timer = None

                    # Enough time has passed, execute the function
                    last_call_time = current_time

                    # if we have accumulated data, we get a compound value
                    if len(accumulator):
                        accumulator.add(*args, **kwargs)
                        acc_args, acc_kwargs = accumulator.get()  # accumulator resets here
                        return func(*acc_args, **acc_kwargs)

                    # No accumulated data, normal call
                    return func(*args, **kwargs)

                else:
                    # Too soon, skip this call
                    accumulator.add(*args, **kwargs)

                    # Schedule execution for when the interval expires
                    if timer is not None:
                        timer.cancel()

                    time_to_wait = min_interval - time_since_last
                    timer = threading.Timer(time_to_wait, execute_accumulated)
                    timer.start()

                    return None

        return wrapper

    return decorator
