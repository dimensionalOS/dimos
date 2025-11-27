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

import traceback


class AgentMemoryError(Exception):
    """
    Base class for all exceptions raised by AgentMemory operations.
    All custom exceptions related to AgentMemory should inherit from this class.

    Args:
        message (str): Human-readable message describing the error.
    """

    def __init__(self, message: str = "Error in AgentMemory operation") -> None:
        super().__init__(message)


class AgentMemoryConnectionError(AgentMemoryError):
    """
    Exception raised for errors attempting to connect to the database.
    This includes failures due to network issues, authentication errors, or incorrect connection parameters.

    Args:
        message (str): Human-readable message describing the error.
        cause (Exception, optional): Original exception, if any, that led to this error.
    """

    def __init__(self, message: str = "Failed to connect to the database", cause=None) -> None:  # type: ignore[no-untyped-def]
        super().__init__(message)
        if cause:
            self.cause = cause
        self.traceback = traceback.format_exc() if cause else None

    def __str__(self) -> str:
        return f"{self.message}\nCaused by: {self.cause!r}" if self.cause else self.message  # type: ignore[attr-defined]


class UnknownConnectionTypeError(AgentMemoryConnectionError):
    """
    Exception raised when an unknown or unsupported connection type is specified during AgentMemory setup.

    Args:
        message (str): Human-readable message explaining that an unknown connection type was used.
    """

    def __init__(
        self, message: str = "Unknown connection type used in AgentMemory connection"
    ) -> None:
        super().__init__(message)


class DataRetrievalError(AgentMemoryError):
    """
    Exception raised for errors retrieving data from the database.
    This could occur due to query failures, timeouts, or corrupt data issues.

    Args:
        message (str): Human-readable message describing the data retrieval error.
    """

    def __init__(
        self, message: str = "Error in retrieving data during AgentMemory operation"
    ) -> None:
        super().__init__(message)


class DataNotFoundError(DataRetrievalError):
    """
    Exception raised when the requested data is not found in the database.
    This is used when a query completes successfully but returns no result for the specified identifier.

    Args:
        vector_id (int or str): The identifier for the vector that was not found.
        message (str, optional): Human-readable message providing more detail. If not provided, a default message is generated.
    """

    def __init__(self, vector_id, message=None) -> None:  # type: ignore[no-untyped-def]
        message = message or f"Requested data for vector ID {vector_id} was not found."
        super().__init__(message)
        self.vector_id = vector_id
