# Copyright 2026 Dimensional Inc.
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

"""Continuously publish consistent snapshots of a live memory2 database."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
from threading import Event
import time
from typing import Any

from dimos.hosted_data.integrations.memory2_upload import (
    Memory2Upload,
    upload_memory2_dataset,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _database_signature(path: Path) -> str:
    """Track SQLite plus WAL changes without reading a large live database."""
    parts: list[str] = []
    for candidate in (path, Path(f"{path}-wal")):
        try:
            stat = candidate.stat()
            parts.append(f"{candidate.name}:{stat.st_size}:{stat.st_mtime_ns}")
        except FileNotFoundError:
            parts.append(f"{candidate.name}:missing")
    return "|".join(parts)


class ContinuousMemory2Publisher:
    """Upload point-in-time memory2 snapshots whenever the live DB changes."""

    def __init__(
        self,
        *,
        path: str | Path,
        server_url: str,
        owner: str,
        repository: str,
        token: str | None = None,
        dataset: str | None = None,
        state_path: str | Path | None = None,
        interval_seconds: float = 5.0,
        retries: int = 3,
        backoff_seconds: float = 0.5,
    ) -> None:
        if interval_seconds < 0.1:
            raise ValueError("interval_seconds must be at least 0.1")
        self.path = Path(path)
        self.server_url = server_url
        self.owner = owner
        self.repository = repository
        self.token = token
        self.dataset = dataset
        self.state_path = (
            Path(state_path)
            if state_path is not None
            else self.path.with_name(f".{self.path.name}.hosted-sync.json")
        )
        self.interval_seconds = interval_seconds
        self.retries = retries
        self.backoff_seconds = backoff_seconds

    def _saved_signature(self) -> str | None:
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            if (
                data.get("server_url") == self.server_url
                and data.get("owner") == self.owner
                and data.get("repository") == self.repository
            ):
                return str(data["source_signature"])
        except (OSError, KeyError, TypeError, json.JSONDecodeError):
            pass
        return None

    def publish_if_changed(self, *, force: bool = False) -> Memory2Upload | None:
        """Publish one consistent snapshot unless the DB and WAL are unchanged."""
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        signature = _database_signature(self.path)
        if not force and signature == self._saved_signature():
            return None
        result = upload_memory2_dataset(
            path=self.path,
            server_url=self.server_url,
            owner=self.owner,
            repository=self.repository,
            token=self.token,
            dataset=self.dataset,
            retries=self.retries,
            backoff_seconds=self.backoff_seconds,
        )
        self.state_path.write_text(
            json.dumps(
                {
                    "version": 1,
                    "server_url": self.server_url,
                    "owner": self.owner,
                    "repository": self.repository,
                    "source_signature": signature,
                    "dataset_object": result.dataset_object.to_dict(),
                    "index_object": result.index_object.to_dict(),
                    "published_at": time.time(),
                },
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        return result

    def run(
        self,
        stop: Event,
        *,
        on_publish: Callable[[Memory2Upload], Any] | None = None,
    ) -> None:
        """Watch until stopped, retrying later after transient publish failures."""
        while not stop.is_set():
            try:
                result = self.publish_if_changed()
                if result is not None and on_publish is not None:
                    on_publish(result)
            except Exception:
                logger.exception("continuous memory2 publish failed; retrying")
            stop.wait(self.interval_seconds)
