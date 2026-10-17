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

import re
import subprocess

from dimos.hosted import service
from dimos.hosted.daemon import DEFAULT_LISTEN, port_free

description = "The default Host port is free or held by this Host"
# Another router there only moves this Host to a fallback port.
warning = True


def check() -> bool:
    host, _, port = DEFAULT_LISTEN.partition("/")[2].rpartition(":")
    local = service.local_host()
    if local is not None and any(e.endswith(f":{port}") for e in local.get("listen", [])):
        return True
    if port_free(host, int(port)):
        return True
    out = subprocess.run(
        ["ss", "-ltnpH", f"sport = :{port}"], capture_output=True, text=True
    ).stdout
    owner = re.search(r'users:\(\("([^"]+)",pid=(\d+)', out)
    who = f"{owner.group(1)} (pid {owner.group(2)})" if owner else "another user's process"
    fallback = local["listen"][0] if local else "a fallback port"
    raise RuntimeError(f"port {port} is held by {who}, not a dimos Host; this Host uses {fallback}")
