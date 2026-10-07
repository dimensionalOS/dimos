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

"""`python -m dimos.gateway.msgs [--write]`: check (or regenerate) msgs.ts and msgs.js; warn about hand-written messages."""

import sys

from dimos.gateway import msgs

found = msgs.scan()
for line in msgs.warnings(found):
    print(f"warning: no LCM schema: {line}", file=sys.stderr)
if "--write" in sys.argv[1:]:
    msgs.write(found)
    print(f"wrote {msgs.TS_FILE.name} and {msgs.JS_FILE.name}: {len(found.schemas)} message types")
else:
    problems = msgs.stale_problems(found)
    print("\n".join(problems) or "msgs.ts and msgs.js are current")
    raise SystemExit(1 if problems else 0)
