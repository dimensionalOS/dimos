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

# Central location for installer execution flags.

from __future__ import annotations

from typing import Any

# Defaults mirror previous behavior; can be updated at runtime.
installer_status: dict[str, Any] = {
    "dry_run": False,  # can be set via CLI
    "dev": True,  # can be set via CLI in the future
    "non_interactive": False,  # set by __main__ when detected/passed
    "template_repo": False,  # set when invoked from a dimos template repo
    "features": [],  # selected features list
}
