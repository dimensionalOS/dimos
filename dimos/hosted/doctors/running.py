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
from dimos.hosted.discovery import probe

description = "The local Host is running and answers on its router"


def check() -> bool:
    local = service.local_host()
    return local is not None and probe(str(local["client_endpoint"])) is not None


def fix() -> None:
    service.start()
    deadline = time.monotonic() + 30.0
    while not check() and time.monotonic() < deadline:
        time.sleep(1.0)
