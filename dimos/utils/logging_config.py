"""Logging configuration module with color support.

This module sets up a logger with color output for different log levels.
"""

import os
import logging
import colorlog
from typing import Optional


def setup_logger(
    name: str, 
    level: Optional[int] = None,
    log_format: Optional[str] = None
) -> logging.Logger:
    """Set up a logger with color output.

    Args:
        name: The name of the logger.
        level: The logging level (e.g., logging.INFO, logging.DEBUG).
               If None, will use DIMOS_LOG_LEVEL env var or default to INFO.
        log_format: Optional custom log format.

    Returns:
        A configured logger instance.
    """
    if level is None:
        # Get level from environment variable or default to INFO
        level_name = os.getenv('DIMOS_LOG_LEVEL', 'INFO')
        level = getattr(logging, level_name)
        
    if log_format is None:
        log_format = "%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    try:
        # Get or create logger
        logger = logging.getLogger(name)
        
        # Remove any existing handlers to avoid duplicates
        if logger.hasHandlers():
            logger.handlers.clear()
            
        # Set logger level first
        logger.setLevel(level)
        
        # Ensure we're not blocked by parent loggers
        logger.propagate = False
        
        # Create and configure handler
        handler = colorlog.StreamHandler()
        handler.setLevel(level)  # Explicitly set handler level
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
        logger.addHandler(handler)

        return logger
    except Exception as e:
        logging.error(f"Failed to set up logger: {e}")
        raise


# Initialize the logger for this module using environment variable
logger = setup_logger(__name__)

# Example usage:
# logger.debug("This is a debug message")
