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

"""Bootstrap an IsolatedPythonModule runtime subclass."""

from __future__ import annotations

from importlib.metadata import EntryPoint
import inspect
import os
import pickle
import signal
import sys
import threading
from typing import Any

import typer

from dimos.core.isolated_python_module import IsolatedPythonModule, contract_rpc_names
from dimos.spec.utils import _signatures_compatible


def load_class(reference: str) -> type[Any]:
    entry_point = EntryPoint(name="isolated-python", value=reference, group="dimos.runtime")
    try:
        if not entry_point.module or not entry_point.attr:
            raise ValueError(f"Invalid import reference {reference!r}; use module:Class")
    except AttributeError as error:
        raise ValueError(f"Invalid import reference {reference!r}; use module:Class") from error

    value = entry_point.load()
    if not isinstance(value, type):
        raise TypeError(f"Import reference {reference!r} does not resolve to a class")
    return value


def _method_owner(module_class: type[Any], name: str) -> type[Any] | None:
    return next((base for base in module_class.__mro__ if name in base.__dict__), None)


def validate_runtime(
    declaration: type[IsolatedPythonModule], runtime: type[IsolatedPythonModule]
) -> None:
    if not issubclass(declaration, IsolatedPythonModule):
        raise TypeError(f"{declaration.__name__} is not an IsolatedPythonModule contract")
    if not issubclass(runtime, declaration):
        raise TypeError(f"{runtime.__name__} must subclass {declaration.__name__}")

    for name in contract_rpc_names(declaration):
        owner = _method_owner(runtime, name)
        if owner is None or owner is declaration or owner in declaration.__mro__[1:]:
            raise TypeError(f"{runtime.__name__} must override contract RPC {name!r}")
        contract_method = getattr(declaration, name)
        runtime_method = getattr(runtime, name)
        if not hasattr(runtime_method, "__rpc__"):
            raise TypeError(f"{runtime.__name__}.{name} must retain @rpc or @skill")
        if bool(hasattr(contract_method, "__skill__")) != bool(
            hasattr(runtime_method, "__skill__")
        ):
            raise TypeError(f"{runtime.__name__}.{name} must preserve skill classification")
        if not _signatures_compatible(
            inspect.signature(contract_method, eval_str=True),
            inspect.signature(runtime_method, eval_str=True),
        ):
            raise TypeError(f"{runtime.__name__}.{name} has an incompatible signature")


def main(
    declaration: str = typer.Option(..., "--declaration"),
    implementation: str = typer.Option(..., "--implementation"),
    instance_name: str = typer.Option(..., "--instance-name"),
    handshake_fd: int = typer.Option(..., "--handshake-fd"),
) -> None:
    module: IsolatedPythonModule | None = None
    try:
        declaration_class = load_class(declaration)
        runtime_class = load_class(implementation)
        if not issubclass(declaration_class, IsolatedPythonModule):
            raise TypeError(f"{declaration!r} is not an IsolatedPythonModule contract")
        if not issubclass(runtime_class, IsolatedPythonModule):
            raise TypeError(f"{implementation!r} is not an IsolatedPythonModule subclass")
        validate_runtime(declaration_class, runtime_class)
        kwargs = pickle.load(sys.stdin.buffer)
        kwargs["instance_name"] = instance_name
        module = runtime_class(_isolated_python_runtime=True, **kwargs)
        os.write(handshake_fd, b"READY\n")
    except Exception as error:
        try:
            os.write(handshake_fd, f"ERROR {type(error).__name__}: {error}\n".encode())
        except OSError:
            pass
        raise
    finally:
        try:
            os.close(handshake_fd)
        except OSError:
            pass

    stopping = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    stopping.wait()
    if module is not None:
        module.stop()


if __name__ == "__main__":
    typer.run(main)
