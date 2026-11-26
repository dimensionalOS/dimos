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

"""Lifecycle adapter for the existing network observer inside the host gateway."""

from __future__ import annotations

import logging
from pathlib import Path
import threading

from dimos.diagnostics.network import monitor
from dimos.diagnostics.session import SessionSettings


class SessionNetwork:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.run_id: str | None = None

    def start(self, root: Path, settings: SessionSettings) -> None:
        if settings.network is None:
            return
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                if self.run_id != settings.run_id:
                    raise ValueError("previous network observer is still stopping")
                return
            self.stop_event = threading.Event()
            self.run_id = settings.run_id
            network = settings.network

            def observe() -> None:
                try:
                    monitor(
                        root,
                        settings.run_id,
                        86400,
                        network.interval_s,
                        interface=network.interface,
                        peers=tuple(network.peers),
                        probe_url=network.probe_url,
                        stop=self.stop_event,
                    )
                except Exception:
                    logging.getLogger(__name__).exception("Diagnostic network observer stopped")

            self.thread = threading.Thread(target=observe, name="diagnostic-network", daemon=True)
            self.thread.start()

    def stop(self) -> None:
        with self.lock:
            self.stop_event.set()
            if self.thread is not None:
                self.thread.join(timeout=10)
