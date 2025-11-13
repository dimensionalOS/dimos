"""Thread pool functionality for parallel execution in the Dimos framework.

This module provides a shared ThreadPoolExecutor exposed through a
ReactiveX scheduler, ensuring consistent thread management across the application.
"""

import os
import multiprocessing
from reactivex.scheduler import ThreadPoolScheduler
from .logging_config import logger


def get_max_workers() -> int:
    """Determine the number of workers for the thread pool.

    Returns:
        int: The number of workers, configurable via the DIMOS_MAX_WORKERS
        environment variable, defaulting to 4 times the CPU count.
    """
    return int(os.getenv('DIMOS_MAX_WORKERS', multiprocessing.cpu_count() * 4))


# Create a ThreadPoolScheduler with a configurable number of workers.
try:
    max_workers = get_max_workers()
    scheduler = ThreadPoolScheduler(max_workers=max_workers)
    logger.info(f"Using {max_workers} workers")
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
