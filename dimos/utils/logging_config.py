# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Mapping
from datetime import datetime
import inspect
import logging
import logging.handlers
import os
from pathlib import Path
import sys
import tempfile
import traceback
from types import TracebackType
from typing import Any

import structlog
from structlog.processors import CallsiteParameter, CallsiteParameterAdder

from dimos.constants import DIMOS_LOG_DIR, DIMOS_PROJECT_ROOT

# Suppress noisy loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)

_LOG_FILE_PATH = None


def _get_log_directory() -> Path:
    # Check if running from a git repository
    if (DIMOS_PROJECT_ROOT / ".git").exists():
        log_dir = DIMOS_LOG_DIR
    else:
        # Running from an installed package - use XDG_STATE_HOME
        xdg_state_home = os.getenv("XDG_STATE_HOME")
        if xdg_state_home:
            log_dir = Path(xdg_state_home) / "dimos" / "logs"
        else:
            log_dir = Path.home() / ".local" / "state" / "dimos" / "logs"

    try:
        log_dir.mkdir(parents=True, exist_ok=True)
    except (PermissionError, OSError):
        log_dir = Path(tempfile.gettempdir()) / "dimos" / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)

    return log_dir


def _get_log_file_path() -> Path:
    log_dir = _get_log_directory()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pid = os.getpid()
    return log_dir / f"dimos_{timestamp}_{pid}.jsonl"


def _configure_structlog() -> Path:
    global _LOG_FILE_PATH

    if _LOG_FILE_PATH:
        return _LOG_FILE_PATH

    _LOG_FILE_PATH = _get_log_file_path()

    shared_processors: list[Any] = [
        structlog.stdlib.add_log_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.UnicodeDecoder(),
        CallsiteParameterAdder(
            parameters=[
                CallsiteParameter.FUNC_NAME,
                CallsiteParameter.LINENO,
            ]
        ),
        structlog.processors.format_exc_info,  # Add this to format exception info
    ]

    structlog.configure(
        processors=[
            structlog.stdlib.filter_by_level,
            *shared_processors,
            structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
        ],
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )

    return _LOG_FILE_PATH


_CONSOLE_PATH_WIDTH = 30
_CONSOLE_USE_COLORS = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

_CONSOLE_LEVEL_COLORS = {
    "dbg": "\033[1;36;40m",  # bold cyan on black
    "inf": "\033[1;32;40m",  # bold green on black
    "war": "\033[1;33;40m",  # bold yellow on black
    "err": "\033[1;31;40m",  # bold red on black
    "cri": "\033[1;31;40m",  # bold red on black
}
_CONSOLE_RESET = "\033[0m"
_CONSOLE_FIXED = "\033[1;30;40m"  # bold dark gray on black
_CONSOLE_TEXT = "\033[0;34m"  # blue
_CONSOLE_KEY = "\033[0;36m"  # cyan
_CONSOLE_VAL = "\033[0;35m"  # magenta
_CONSOLE_EQ = "\033[0;37m"  # white


def _compact_console_processor(logger: Any, method_name: str, event_dict: Mapping[str, Any]) -> str:
    """Format log lines as: HH:MM:SS.mmm[lvl][file.py              ] Event key=value ..."""
    event_dict = dict(event_dict)

    # Time — HH:MM:SS.mmm
    timestamp = event_dict.pop("timestamp", "")
    if timestamp:
        try:
            dt = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
            time_str = dt.strftime("%H:%M:%S") + f".{dt.microsecond // 1000:03d}"
        except (ValueError, AttributeError):
            time_str = str(timestamp)[:12]
    else:
        now = datetime.now()
        time_str = now.strftime("%H:%M:%S") + f".{now.microsecond // 1000:03d}"

    # Level — 3-letter lowercase abbreviation
    level = event_dict.pop("level", "???")
    level_short = level[:3].lower()

    # File path — fixed width, truncated from the left, padded on the right
    file_path = event_dict.pop("logger", "")
    if len(file_path) > _CONSOLE_PATH_WIDTH:
        file_path = file_path[-_CONSOLE_PATH_WIDTH:]
    file_path = f"{file_path:<{_CONSOLE_PATH_WIDTH}s}"

    # Event message
    event = event_dict.pop("event", "")

    # Remove internal / callsite / exception fields
    for key in (
        "func_name",
        "lineno",
        "exception",
        "exc_info",
        "exception_type",
        "exception_message",
        "traceback_lines",
        "_record",
        "_from_structlog",
    ):
        event_dict.pop(key, None)

    # Assemble the line
    if _CONSOLE_USE_COLORS:
        R = _CONSOLE_RESET
        color = _CONSOLE_LEVEL_COLORS.get(level_short, "")
        line = (
            f"{_CONSOLE_FIXED}{time_str}{R}"
            f"{color}[{level_short}]{R}"
            f"{_CONSOLE_FIXED}[{file_path}]{R} "
            f"{_CONSOLE_TEXT}{event}{R}"
        )
        if event_dict:
            kv_parts = " ".join(
                f"{_CONSOLE_KEY}{k}{_CONSOLE_EQ}={_CONSOLE_VAL}{v}{R}"
                for k, v in sorted(event_dict.items())
            )
            line += " " + kv_parts
    else:
        kv_str = " ".join(f"{k}={v}" for k, v in sorted(event_dict.items()))
        line = f"{time_str} [{level_short}][{file_path}] {event}"
        if kv_str:
            line += " " + kv_str

    return line


def setup_logger(*, level: int | None = None) -> Any:
    """Set up a structured logger using structlog.

    Args:
        level: The logging level.

    Returns:
        A configured structlog logger instance.
    """

    caller_frame = inspect.stack()[1]
    name = caller_frame.filename

    # Convert absolute path to relative path
    try:
        name = str(Path(name).relative_to(DIMOS_PROJECT_ROOT))
    except (ValueError, TypeError):
        pass

    log_file_path = _configure_structlog()

    if level is None:
        level_name = os.getenv("DIMOS_LOG_LEVEL", "INFO")
        level = getattr(logging, level_name)

    stdlib_logger = logging.getLogger(name)

    # Remove any existing handlers.
    if stdlib_logger.hasHandlers():
        stdlib_logger.handlers.clear()

    stdlib_logger.setLevel(level)
    stdlib_logger.propagate = False

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_formatter = structlog.stdlib.ProcessorFormatter(
        processor=_compact_console_processor,
    )
    console_handler.setFormatter(console_formatter)
    stdlib_logger.addHandler(console_handler)

    # Create rotating file handler with JSON formatting.
    file_handler = logging.handlers.RotatingFileHandler(
        log_file_path,
        mode="a",
        maxBytes=10 * 1024 * 1024,  # 10MiB
        backupCount=20,
        encoding="utf-8",
    )
    file_handler.setLevel(level)
    file_formatter = structlog.stdlib.ProcessorFormatter(
        processor=structlog.processors.JSONRenderer(),
    )
    file_handler.setFormatter(file_formatter)
    stdlib_logger.addHandler(file_handler)

    return structlog.get_logger(name)


def setup_exception_handler() -> None:
    def handle_exception(
        exc_type: type[BaseException],
        exc_value: BaseException,
        exc_traceback: TracebackType | None,
    ) -> None:
        # Don't log KeyboardInterrupt
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        # Get a logger for uncaught exceptions
        logger = setup_logger()

        # Log the exception with full traceback to JSON
        logger.error(
            "Uncaught exception occurred",
            exc_info=(exc_type, exc_value, exc_traceback),
            exception_type=exc_type.__name__,
            exception_message=str(exc_value),
            traceback_lines=traceback.format_exception(exc_type, exc_value, exc_traceback),
        )

        # Still display the exception nicely on console using Rich if available
        try:
            from rich.console import Console
            from rich.traceback import Traceback

            console = Console()
            tb = Traceback.from_exception(exc_type, exc_value, exc_traceback)
            console.print(tb)
        except ImportError:
            # Fall back to standard exception display if Rich is not available
            sys.__excepthook__(exc_type, exc_value, exc_traceback)

    # Set our custom exception handler
    sys.excepthook = handle_exception
