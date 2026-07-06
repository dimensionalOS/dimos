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

"""`dimos spy` TUI: live table of all topics across all pubsub transports.

Textual app modeled on run_lcmspy.py (DataTable, 0.5s refresh, theme colors,
'q' to quit, optional `spy web` mode via textual-serve). One row per
(transport, topic) from TransportSpy.snapshot():

    Transport | Topic | Type | Freq (Hz) | Bandwidth | Total | Age

- Topic/Type come from split_type_suffix(); rows sort by total traffic.
- Age is seconds since TopicStats.last_seen (liveness: dims/greys out stale rows).
- `--transport lcm --transport zenoh` (repeatable) filters sources; default all.
- LCM system config warning: call lcmservice.autoconf(check_only=True) before
  entering raw TUI mode, like run_lcmspy does.
"""

from __future__ import annotations


def main() -> None:
    """Entry point for `dimos spy` (argv: optional 'web', --transport filters)."""
    raise NotImplementedError
