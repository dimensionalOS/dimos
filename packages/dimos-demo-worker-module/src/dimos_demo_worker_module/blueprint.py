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

from pathlib import Path
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.runtime_environment import PythonVenvRuntimeEnvironment

VENV_DEMO_ENV_NAME = "dimos-demo-venv"


class VenvDemoPublisher(Module):
    @rpc
    def render_message(self, message: str = "hello") -> str:
        from dimos_demo_worker_module.runtime_dependency import decorate_message

        return decorate_message(message)


class VenvDemoConsumer(Module):
    publisher: VenvDemoPublisher

    @rpc
    def consume_message(self, message: str = "hello") -> str:
        return self.publisher.render_message(message)


def venv_demo_blueprint(
    runtime: PythonVenvRuntimeEnvironment | None = None,
):
    environment = runtime or PythonVenvRuntimeEnvironment(
        name=VENV_DEMO_ENV_NAME,
        python_executable=Path(sys.executable),
    )
    return autoconnect(
        VenvDemoPublisher.blueprint(),
        VenvDemoConsumer.blueprint(),
    ).runtime_environments(environment).runtime_placements({VenvDemoPublisher: environment.name})
