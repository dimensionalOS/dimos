"""
Robot Web Interface wrapper for DIMOS.
Provides a clean interface to the dimensional-interface FastAPI server.
"""

import os
import sys

# Add the dimos-interface api directory to path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(CURRENT_DIR, "dimos-interface/api"))

from dimos.web.fastapi_server import FastAPIServer

class RobotWebInterface(FastAPIServer):
    """Wrapper class for the dimos-interface FastAPI server."""
    
    def __init__(self, port=5555, **streams):
        super().__init__(
            dev_name="Robot Web Interface",
            edge_type="Bidirectional",
            host="0.0.0.0",
            port=port,
            **streams
        ) 