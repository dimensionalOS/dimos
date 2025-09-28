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

import asyncio
import threading
import pytest
import csv
import os
from datetime import datetime


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(autouse=True)
def monitor_threads(request):
    test_name = request.node.name
    test_module = request.node.module.__name__
    initial_threads = threading.active_count()
    initial_thread_names = [t.name for t in threading.enumerate()]
    start_time = datetime.now()

    yield

    end_time = datetime.now()
    final_threads = threading.active_count()
    final_thread_names = [t.name for t in threading.enumerate()]

    new_threads = [t for t in final_thread_names if t not in initial_thread_names]
    dead_threads = [t for t in initial_thread_names if t not in final_thread_names]
    leaked_threads = final_threads - initial_threads

    csv_file = "thread_monitor_report.csv"
    file_exists = os.path.isfile(csv_file)

    with open(csv_file, "a", newline="") as f:
        writer = csv.writer(f)

        if not file_exists:
            writer.writerow(
                [
                    "timestamp",
                    "test_module",
                    "test_name",
                    "initial_threads",
                    "final_threads",
                    "thread_change",
                    "leaked_threads",
                    "new_thread_names",
                    "closed_thread_names",
                    "duration_seconds",
                ]
            )

        writer.writerow(
            [
                start_time.isoformat(),
                test_module,
                test_name,
                initial_threads,
                final_threads,
                final_threads - initial_threads,
                leaked_threads,
                "|".join(new_threads) if new_threads else "",
                "|".join(dead_threads) if dead_threads else "",
                (end_time - start_time).total_seconds(),
            ]
        )
