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

"""
Hello World Docker Module
==========================

Minimal example showing a DimOS module running inside Docker.

The module receives a string on its ``prompt`` input stream, runs it through
cowsay inside the container, and publishes the ASCII art on its ``greeting``
output stream.

NOTE: Requires Linux. Docker Desktop on macOS does not support host networking,
which is needed for LCM multicast between host and container.

Usage:
    python examples/docker_hello_world/hello_docker.py
"""

from __future__ import annotations

from dataclasses import field
from pathlib import Path
import subprocess
import time

from reactivex.disposable import Disposable

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.docker_module import DockerModuleConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out


class HelloDockerConfig(DockerModuleConfig):
    docker_image: str = "dimos-hello-docker:latest"
    docker_file: Path | None = Path(__file__).parent / "Dockerfile"
    docker_build_context: Path | None = Path(__file__).parents[2]  # repo root
    docker_gpus: str | None = None  # no GPU needed
    docker_rm: bool = True
    docker_restart_policy: str = "no"
    docker_env: dict[str, str] = field(default_factory=lambda: {"CI": "1"})

    # Custom (non-docker) config field — passed to the container via JSON
    greeting_prefix: str = "Hello"


class HelloDockerModule(Module["HelloDockerConfig"]):
    """A trivial module that runs inside Docker and echoes greetings."""

    default_config = HelloDockerConfig
    deployment = "docker"

    prompt: In[str]
    greeting: Out[str]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.prompt.subscribe(self._on_prompt)))

    def _cowsay(self, text: str) -> str:
        """Run cowsay inside the container and return the ASCII art."""
        return subprocess.check_output(["cowsay", text], text=True)

    def _on_prompt(self, text: str) -> None:
        art = self._cowsay(text)
        print(f"[HelloDockerModule]\n{art}")
        self.greeting.publish(art)

    @rpc
    def greet(self, name: str) -> str:
        """RPC method that can be called directly."""
        prefix = self.config.greeting_prefix
        return self._cowsay(f"{prefix}, {name}!")

    @rpc
    def get_greeting_prefix(self) -> str:
        """Return the config value to verify it was passed to the container."""
        return self.config.greeting_prefix


class PromptModule(Module):
    """Publishes prompts and listens to greetings."""

    prompt: Out[str]
    greeting: In[str]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.greeting.subscribe(self._on_greeting)))

    @rpc
    def send(self, text: str) -> None:
        """Publish a prompt message onto the stream."""
        self.prompt.publish(text)

    def _on_greeting(self, text: str) -> None:
        print(f"[PromptModule] Received: {text}")


if __name__ == "__main__":
    coordinator = autoconnect(
        PromptModule.blueprint(),
        HelloDockerModule.blueprint(greeting_prefix="Howdy"),
    ).build()

    # Get module proxies
    prompt_mod = coordinator.get_instance(PromptModule)
    docker_mod = coordinator.get_instance(HelloDockerModule)

    # Test that custom config was passed to the container
    prefix = docker_mod.get_greeting_prefix()
    assert prefix == "Howdy", f"Expected 'Howdy', got {prefix!r}"
    print(f"Config passed to container: greeting_prefix={prefix!r}")

    # Test RPC (should use the custom prefix)
    print(docker_mod.greet("World"))

    # Test stream
    prompt_mod.send("stream test")
    time.sleep(2)

    coordinator.stop()
    print("Done!")
