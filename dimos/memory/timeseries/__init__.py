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
"""Time series storage and replay."""

from dimos.memory.timeseries.base import TimeSeriesStore
from dimos.memory.timeseries.inmemory import InMemoryStore
from dimos.memory.timeseries.pickledir import PickleDirStore
from dimos.memory.timeseries.sqlite import SqliteStore


def __getattr__(name: str):  # type: ignore[no-untyped-def]
    if name == "PostgresStore":
        from dimos.memory.timeseries.postgres import PostgresStore

        return PostgresStore
    if name == "reset_db":
        from dimos.memory.timeseries.postgres import reset_db

        return reset_db
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "InMemoryStore",
    "PickleDirStore",
    "PostgresStore",
    "SqliteStore",
    "TimeSeriesStore",
    "reset_db",
]
