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

from contextlib import ExitStack

import pytest

from dimos.core.stream import Out, Transport
from dimos.core.transport_factory import rpc_backend
from dimos.teleop.policy.module import PolicyTeleopModule


@pytest.fixture
def modules(mocker):
    backend = rpc_backend()
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(backend, method)
    with ExitStack() as stack:

        def make(cls=PolicyTeleopModule, **kwargs):
            module = cls(**kwargs)
            stack.callback(module.stop)
            for port in module.inputs.values():
                source = Out(type=port.type, name=port.name)
                transport = mocker.Mock(spec=Transport)

                def subscribe(callback, stream, source=source, transport=transport):
                    unsubscribe = mocker.Mock(wraps=source.subscribe(callback))
                    transport.subscribe.return_value = unsubscribe
                    return unsubscribe

                transport.subscribe.side_effect = subscribe
                transport.publish.side_effect = source.publish
                port.transport = transport
            return module

        yield make
