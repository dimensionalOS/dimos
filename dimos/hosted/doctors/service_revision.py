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

import time

from dimos.hosted import service
from dimos.hosted.daemon import code_revision
from dimos.hosted.discovery import probe

description = "The running Host serves this checkout's code revision"


def check() -> bool:
    local = service.local_host()
    found = probe(str(local["client_endpoint"])) if local else None
    if found is None:
        return True  # nothing running: service_running reports that
    mine = [h for h in found.hosts if h.host_id == local["host_id"]]  # type: ignore[index]
    return bool(mine) and mine[0].versions.get("application_revision") == code_revision()


def fix() -> None:
    service.restart()
    deadline = time.monotonic() + 30.0
    while not check() and time.monotonic() < deadline:
        time.sleep(1.0)
