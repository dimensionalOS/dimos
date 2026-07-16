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

import base64
import os
import pickle
import signal
import threading
import time

import typer

from dimos.core.external_python_module import ExternalPythonModule
from dimos.core.module import Module


def _load(ref: str) -> type:
    module_name, separator, class_name = ref.partition(":")
    if not separator:
        module_name, _, class_name = ref.rpartition(".")
    if not module_name or not class_name:
        raise ValueError(f"Invalid import reference {ref!r}; use module:Class")
    try:
        module = __import__(module_name, fromlist=[class_name])
        value = getattr(module, class_name)
    except (ImportError, AttributeError) as error:
        raise ImportError(
            f"Could not load class {class_name!r} from {module_name!r}: {error}"
        ) from error
    if not isinstance(value, type):
        raise TypeError(f"Import reference {ref!r} does not resolve to a class")
    return value


def main(
    declaration: str = typer.Option(..., "--declaration"),
    implementation: str = typer.Option(..., "--implementation"),
    handshake_fd: int = typer.Option(..., "--handshake-fd"),
    kwargs: str = typer.Option(..., "--kwargs"),
) -> None:
    handshake_open = True
    try:
        declaration_class = _load(declaration)
        implementation_class = _load(implementation)
        if not issubclass(declaration_class, ExternalPythonModule):
            raise TypeError(f"Declaration {declaration!r} is not an ExternalPythonModule")
        if not issubclass(implementation_class, Module):
            raise TypeError(f"Implementation {implementation!r} is not a Module subclass")
        if not issubclass(implementation_class, declaration_class):
            raise TypeError(f"Implementation {implementation!r} does not implement {declaration!r}")
        module_kwargs = pickle.loads(base64.b64decode(kwargs))
        module = implementation_class(**module_kwargs)
        if module.rpc is None:
            raise RuntimeError(f"Implementation {implementation!r} has no RPC server")
        module.rpc.serve_module_rpc(module, name=declaration_class.__name__)
        os.write(handshake_fd, b"READY\n")
    except Exception as error:
        handshake_open = False
        try:
            os.write(
                handshake_fd,
                f"ERROR {type(error).__name__}: {error}\n".encode(),
            )
        except OSError:
            handshake_open = False
        raise
    finally:
        if handshake_open:
            try:
                os.close(handshake_fd)
            except OSError:
                pass

    stopping = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while not stopping.is_set():
        time.sleep(1)


if __name__ == "__main__":
    typer.run(main)
