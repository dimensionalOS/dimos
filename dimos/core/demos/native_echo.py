#!/usr/bin/env python3
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

"""Echo binary for NativeModule tests.

Parses --output_file and --die_after from CLI args, writes remaining
args as JSON to the output file, then waits for SIGTERM.
"""

import argparse
import json
import signal
import sys
import time

print("this message goes to stdout")
print("this message goes to stderr", file=sys.stderr)

signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

parser = argparse.ArgumentParser()
parser.add_argument("--output_file", default=None)
parser.add_argument("--die_after", type=float, default=None)
args, _ = parser.parse_known_args()

if args.output_file:
    with open(args.output_file, "w") as f:
        json.dump(sys.argv[1:], f)

print("my args:", json.dumps(sys.argv[1:]))

if args.die_after is not None:
    time.sleep(args.die_after)
    sys.exit(42)

signal.pause()
