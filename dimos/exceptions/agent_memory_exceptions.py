import traceback

class AgentMemoryError(Exception):
    """
    Base class for all exceptions raised by AgentMemory operations.
    All custom exceptions related to AgentMemory should inherit from this class.
    
    Args:
        message (str): Human-readable message describing the error.
    """
    def __init__(self, message="Error in AgentMemory operation"):
        super().__init__(message)

class AgentMemoryConnectionError(AgentMemoryError):
    """
    Exception raised for errors attempting to connect to the database.
    This includes failures due to network issues, authentication errors, or incorrect connection parameters.
    
    Args:
        message (str): Human-readable message describing the error.
        cause (Exception, optional): Original exception, if any, that led to this error.
    """
    def __init__(self, message="Failed to connect to the database", cause=None):
        super().__init__(message)
        if cause:
            self.cause = cause
        self.traceback = traceback.format_exc() if cause else None

    def __str__(self):
        return f"{self.message}\nCaused by: {repr(self.cause)}" if self.cause else self.message

class UnknownConnectionTypeError(AgentMemoryConnectionError):
    """
    Exception raised when an unknown or unsupported connection type is specified during AgentMemory setup.
    
    Args:
        message (str): Human-readable message explaining that an unknown connection type was used.
    """
    def __init__(self, message="Unknown connection type used in AgentMemory connection"):
        super().__init__(message)

class DataRetrievalError(AgentMemoryError):
    """
    Exception raised for errors retrieving data from the database.
    This could occur due to query failures, timeouts, or corrupt data issues.
    
    Args:
        message (str): Human-readable message describing the data retrieval error.
    """
    def __init__(self, message="Error in retrieving data during AgentMemory operation"):
        super().__init__(message)

class DataNotFoundError(DataRetrievalError):
    """
    Exception raised when the requested data is not found in the database.
    This is used when a query completes successfully but returns no result for the specified identifier.
    
    Args:
        vector_id (int or str): The identifier for the vector that was not found.
        message (str, optional): Human-readable message providing more detail. If not provided, a default message is generated.
    """
    def __init__(self, vector_id, message=None):
        message = message or f"Requested data for vector ID {vector_id} was not found."
        super().__init__(message)
        self.vector_id = vector_id
