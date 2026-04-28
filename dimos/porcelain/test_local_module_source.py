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

from __future__ import annotations

from dimos.porcelain.local_module_source import LocalModuleSource


def test_is_not_remote(running_app):
    assert isinstance(running_app._source, LocalModuleSource)
    assert running_app._source.is_remote is False


def test_list_module_names(running_app):
    names = running_app._source.list_module_names()
    assert "StressTestModule" in names


def test_get_rpyc_module(running_app):
    module = running_app._source.get_rpyc_module("StressTestModule")
    assert module._module_closed is False


def test_get_rpyc_module_caches_connection(running_app):
    source = running_app._source
    m1 = source.get_rpyc_module("StressTestModule")
    m2 = source.get_rpyc_module("StressTestModule")
    assert m1 is m2


def test_invalidate_allows_reconnect(running_app):
    source = running_app._source
    m1 = source.get_rpyc_module("StressTestModule")
    source.invalidate("StressTestModule")
    m2 = source.get_rpyc_module("StressTestModule")
    assert m1 is not m2
    assert m2._module_closed is False


def test_invalidate_unknown_name_is_noop(running_app):
    running_app._source.invalidate("NonexistentModule")


def test_close_then_get_reconnects(running_app):
    source = running_app._source
    m1 = source.get_rpyc_module("StressTestModule")
    source.close()
    m2 = source.get_rpyc_module("StressTestModule")
    assert m1 is not m2
    assert m2._module_closed is False
