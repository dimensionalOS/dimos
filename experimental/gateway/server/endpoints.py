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

"""Load leaf modules whose filenames mirror their literal HTTP paths."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

from fastapi import FastAPI

from experimental.gateway.server.state import ServerState

ENDPOINTS = Path(__file__).parents[1] / "endpoints"


def register_endpoints(app: FastAPI, state: ServerState) -> None:
    inventory = json.loads((ENDPOINTS / "inventory.json").read_text())
    for filename in dict.fromkeys(row[2] for row in inventory):
        name = "experimental.gateway.endpoints." + filename.removesuffix(".py").replace("/", ".")
        spec = importlib.util.spec_from_file_location(name, ENDPOINTS / filename)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load gateway endpoint {filename}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        module.register(app, state)
