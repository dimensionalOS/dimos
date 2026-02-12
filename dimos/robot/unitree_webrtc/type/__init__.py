#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Compatibility re-exports for legacy dimos.robot.unitree_webrtc.type.* imports."""

import importlib

__all__ = []


def __getattr__(name: str):  # type: ignore[no-untyped-def]
    module = importlib.import_module("dimos.robot.unitree.type")
    try:
        return getattr(module, name)
    except AttributeError as exc:
        raise AttributeError(f"No {__name__} attribute {name}") from exc


def __dir__() -> list[str]:
    module = importlib.import_module("dimos.robot.unitree.type")
    return [name for name in dir(module) if not name.startswith("_")]
