"""Logging configuration module with color support.

This module sets up a logger with color output for different log levels.
"""

import logging
import colorlog
from typing import Optional


def setup_logger(
    name: str, level: int = logging.INFO, log_format: Optional[str] = None
) -> logging.Logger:
    """Set up a logger with color output.

    Args:
        name: The name of the logger.
        level: The logging level (e.g., logging.INFO, logging.DEBUG).
        log_format: Optional custom log format.

    Returns:
        A configured logger instance.
    """
    if log_format is None:
        log_format = "%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    try:
        handler = colorlog.StreamHandler()
        formatter = colorlog.ColoredFormatter(
            log_format,
            log_colors={
                "DEBUG": "cyan",
                "INFO": "green",
                "WARNING": "yellow",
                "ERROR": "red",
                "CRITICAL": "bold_red",
            },
        )
        handler.setFormatter(formatter)

        logger = logging.getLogger(name)
        if not logger.hasHandlers():
            logger.addHandler(handler)
            logger.setLevel(level)

        return logger
    except Exception as e:
        logging.error(f"Failed to set up logger: {e}")
        raise


# Initialize the logger for this module
logger = setup_logger(__name__)

# Example usage:
# logger.debug("This is a debug message")
