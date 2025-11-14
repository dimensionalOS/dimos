import logging

from .candidate import Candidate
from .ice import Connection, ConnectionClosed, TransportPolicy

__all__ = ["Candidate", "Connection", "ConnectionClosed", "TransportPolicy"]
__version__ = "0.9.0"

# Set default logging handler to avoid "No handler found" warnings.
logging.getLogger(__name__).addHandler(logging.NullHandler())
