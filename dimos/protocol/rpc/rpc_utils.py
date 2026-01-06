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

"""Utilities for serializing and deserializing exceptions for RPC transport."""

from __future__ import annotations

import traceback
from typing import Any, TypedDict


class SerializedException(TypedDict):
    """Type for serialized exception data."""

    type_name: str
    type_module: str
    args: tuple[Any, ...]
    traceback: str


class RemoteError(Exception):
    """Exception that was raised on a remote RPC server.

    Preserves the original exception type and full stack trace from the remote side.
    """

    def __init__(
        self, type_name: str, type_module: str, args: tuple[Any, ...], traceback: str
    ) -> None:
        super().__init__(*args if args else (f"Remote exception: {type_name}",))
        self.remote_type = f"{type_module}.{type_name}"
        self.remote_traceback = traceback

    def __str__(self) -> str:
        base_msg = super().__str__()
        return (
            f"[Remote {self.remote_type}] {base_msg}\n\nRemote traceback:\n{self.remote_traceback}"
        )


def serialize_exception(exc: Exception) -> SerializedException:
    """Convert an exception to a transferable format.

    Args:
        exc: The exception to serialize

    Returns:
        A dictionary containing the exception data that can be transferred
    """
    # Get the full traceback as a string
    tb_str = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))

    return SerializedException(
        type_name=type(exc).__name__,
        type_module=type(exc).__module__,
        args=exc.args,
        traceback=tb_str,
    )


def deserialize_exception(exc_data: SerializedException) -> Exception:
    """Reconstruct an exception from serialized data.

    For builtin exceptions, instantiates the actual type.
    For custom exceptions, returns a RemoteError.

    Args:
        exc_data: The serialized exception data

    Returns:
        An exception that can be raised with full type and traceback info
    """
    type_name = exc_data.get("type_name", "Exception")
    type_module = exc_data.get("type_module", "builtins")
    args: tuple[Any, ...] = exc_data.get("args", ())
    tb_str = exc_data.get("traceback", "")

    # Only reconstruct builtin exceptions
    if type_module == "builtins":
        try:
            import builtins

            exc_class = getattr(builtins, type_name, None)
            if exc_class and issubclass(exc_class, BaseException):
                exc = exc_class(*args)
                # Add remote traceback as __cause__ for context
                exc.__cause__ = RemoteError(type_name, type_module, args, tb_str)
                return exc  # type: ignore[no-any-return]
        except (AttributeError, TypeError):
            pass

    # Use RemoteError for non-builtin or if reconstruction failed
    return RemoteError(type_name, type_module, args, tb_str)
