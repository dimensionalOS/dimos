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

import asyncio
import threading

import pytest
from reactivex.subject import Subject

from dimos.core.module import Module
from dimos.core.transport_factory import rpc_backend


@pytest.fixture
def module(mocker):
    backend = rpc_backend()
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(backend, method)
    instance = Module()
    try:
        yield instance
    finally:
        instance.stop()


def test_stop_awaits_inflight_handler_cleanup_before_closing_loop(module):
    source = Subject()
    started, cleaned_up = threading.Event(), threading.Event()

    async def handle(value):
        try:
            started.set()
            await asyncio.Event().wait()
        finally:
            await asyncio.sleep(0)
            cleaned_up.set()

    module.process_observable(source, handle)
    source.on_next(1)
    assert started.wait(2)
    module.stop()
    assert cleaned_up.is_set()
