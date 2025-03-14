"""
Robot Web Interface wrapper for DIMOS.
Provides a clean interface to the dimensional-interface FastAPI server.
"""

import os
import sys

from dimos.web.dimos_interface.api.server import FastAPIServer

class RobotWebInterface(FastAPIServer):
    """Wrapper class for the dimos-interface FastAPI server."""
    
    def __init__(self, port=5555, text_streams=None, **streams):
        super().__init__(
            dev_name="Robot Web Interface",
            edge_type="Bidirectional",
            host="0.0.0.0",
            port=port,
            text_streams=text_streams,  # Initialize with at least one text stream
            **streams
        ) 