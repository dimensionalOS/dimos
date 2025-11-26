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

"""
Robot Web Interface wrapper for DIMOS.
Provides a clean interface to the dimensional-interface FastAPI server.
"""

from dimos.web.dimos_interface.api.server import FastAPIServer


class RobotWebInterface(FastAPIServer):
    """Wrapper class for the dimos-interface FastAPI server."""

    def __init__(self, port: int = 5555, text_streams=None, audio_subject=None, **streams) -> None:  # type: ignore[no-untyped-def]
        super().__init__(
            dev_name="Robot Web Interface",
            edge_type="Bidirectional",
            host="0.0.0.0",
            port=port,
            text_streams=text_streams,
            audio_subject=audio_subject,
            **streams,
        )
