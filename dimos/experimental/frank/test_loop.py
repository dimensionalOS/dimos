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

"""Conversational turn cleanup must not own service operations."""

import argparse
import importlib.util
from pathlib import Path

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "frank_loop_regression", Path(__file__).with_name("loop.py")
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


@pytest.mark.parametrize("exit_code,stop_count", [(0, 0), (1, 1)])
def test_only_failed_turn_stops_movement(mocker, exit_code, stop_count):
    mocker.patch.object(_MODULE, "harness_cmd", return_value=["fake-pi"])
    proc = mocker.Mock(pid=123, returncode=exit_code)
    proc.communicate.return_value = ("done", "")
    mocker.patch.object(_MODULE.subprocess, "Popen", return_value=proc)
    mocker.patch.object(_MODULE.psutil, "Process").return_value.children.return_value = []
    mocker.patch.object(_MODULE.os, "killpg")
    stop = mocker.patch.object(_MODULE.subprocess, "run")
    stop.return_value.returncode = 0
    mocker.patch.object(_MODULE, "log")
    args = argparse.Namespace(harness="pi", session="test", model=None, turn_timeout=10)
    _MODULE.run_turn(args, {"type": "chat", "name": "Henry"})
    assert stop.call_count == stop_count
