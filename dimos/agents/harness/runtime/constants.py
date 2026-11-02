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

"""Fixed protocol names, module naming rules, runtime output limits, and the JSON results hold."""

import re
from typing import TypeAlias

from dimos.constants import HARNESS_STATE_DIR

JSON: TypeAlias = "str | int | float | bool | None | list[JSON] | dict[str, JSON]"

HARNESS_EVENT_METHOD = "dimos/harness_event"
HARNESS_STATUS_METHOD = "dimos/harness_status"
CONVERSATION_META = "dimos/conversation"
CONNECT_TIMEOUT_S = 3.0

AGENT_MODULE_PACKAGE = "dimos_agent"
# A name ending in _rN would be the file of another module's revision N.
AGENT_MODULE_NAME = re.compile(r"^(?!.*_r\d+$)[a-z][a-z0-9_]{0,63}$")
AGENT_MODULE_PATH = re.compile(
    rf"^{AGENT_MODULE_PACKAGE}\.(?P<name>[a-z][a-z0-9_]*)_r(?P<revision>\d+)$"
)
# ABCMeta stamps this on every class, so a subclass did not choose it.
METACLASS_NAMES = frozenset({"_abc_impl"})
TRACEBACK_HEADER = "Traceback (most recent call last):"
# The margin an exception group's traceback puts before each line of its members.
EXCEPTION_GROUP_MARGIN = re.compile(r"^\s*\| ?")

# These limits apply to compact inventory facts; the full result holds everything.
INVENTORY_SECTION_LIMIT = 200
INVENTORY_DOC_CHARS = 160
LIFECYCLE_RPCS = frozenset(
    {
        "build",
        "get_skills",
        "get_stream_descriptors",
        "on_system_modules",
        "peek_stream",
        "set_module_ref",
        "set_transport",
        "start",
        "stop",
    }
)

EXPORTS_DIR = HARNESS_STATE_DIR / "exports"
# Point clouds above this size are exported with a uniform stride.
MAX_EXPORT_POINTS = 200_000
MAX_EXPORT_FILES = 50
JPEG_QUALITY = 85
