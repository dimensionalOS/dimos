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

"""Internal tracer state for `dimos.telemetry`.

Owns a module-global `TracerManager` that holds the active OTEL tracer
and an explicit on/off flag. No opentelemetry packages are imported
here; that cost is paid only when a wiring entry point (`enable`,
`configure_tracing`, `DimosInstrumentor`) is called.
"""

from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TracerManager:
    """Holds the active tracer plus an on/off flag.

    Stays inert at import time: the tracer is None and export is off
    until a caller wires us up.
    """

    def __init__(self) -> None:
        self.tracer: Any = None
        self._export_enabled: bool = False

    def configure(self, tracer: Any) -> None:
        self.tracer = tracer
        self._export_enabled = True
        logger.info("dimos.telemetry: tracing configured.")

    def reset(self) -> None:
        self.tracer = None
        self._export_enabled = False


_manager = TracerManager()
