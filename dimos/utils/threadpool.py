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

"""Thread pool functionality for parallel execution in the Dimos framework.

This module provides a shared ThreadPoolExecutor exposed through a
ReactiveX scheduler, ensuring consistent thread management across the application.
"""

import multiprocessing
import os

from reactivex.scheduler import ThreadPoolScheduler

from .logging_config import setup_logger

logger = setup_logger()


def get_max_workers() -> int:
    """Determine the number of workers for the thread pool.

    Returns:
        int: The number of workers, configurable via the DIMOS_MAX_WORKERS
        environment variable, defaulting to 4 times the CPU count.
    """
    env_value = os.getenv("DIMOS_MAX_WORKERS", "")
    return int(env_value) if env_value.strip() else multiprocessing.cpu_count()


# Create a ThreadPoolScheduler with a configurable number of workers.
try:
    max_workers = get_max_workers()
    scheduler = ThreadPoolScheduler(max_workers=max_workers)
    # logger.info(f"Using {max_workers} workers")
except Exception as e:
    logger.error(f"Failed to initialize ThreadPoolScheduler: {e}")
    raise


def get_scheduler() -> ThreadPoolScheduler:
    """Return the global ThreadPoolScheduler instance.

    The thread pool is configured with a fixed number of workers and is shared
    across the application to manage system resources efficiently.

    Returns:
        ThreadPoolScheduler: The global scheduler instance for scheduling
        operations on the thread pool.
    """
    return scheduler


def make_single_thread_scheduler() -> ThreadPoolScheduler:
    """Create a new ThreadPoolScheduler with a single worker.

    This provides a dedicated scheduler for tasks that should run serially
    on their own thread rather than using the shared thread pool.

    Returns:
        ThreadPoolScheduler: A scheduler instance with a single worker thread.
    """
    return ThreadPoolScheduler(max_workers=1)


# Example usage:
# scheduler = get_scheduler()
# # Use the scheduler for parallel tasks
