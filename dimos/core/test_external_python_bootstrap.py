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

import base64
import os
import pickle
import subprocess
import sys

import pytest
import typer
from typer.testing import CliRunner

import dimos.core.external_python_bootstrap as bootstrap


class Declaration(bootstrap.ExternalPythonModule):
    implementation = "unused:Implementation"


class Implementation(Declaration):
    pass


class BrokenImplementation(Declaration):
    def __init__(self, **kwargs: object) -> None:
        raise RuntimeError("constructor failed deliberately")


def test_load_accepts_colon_and_dotted_references() -> None:
    assert bootstrap._load(__name__ + ":Implementation") is Implementation
    assert bootstrap._load(__name__ + ".Implementation") is Implementation


@pytest.mark.parametrize("reference", ["", "Implementation", ":Implementation", "module:"])
def test_load_rejects_invalid_references(reference: str) -> None:
    with pytest.raises(ValueError, match="Invalid import reference"):
        bootstrap._load(reference)


def test_main_rejects_contract_mismatch_before_starting_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class NotImplementation:
        pass

    values = {"decl:Declaration": Declaration, "impl:Implementation": NotImplementation}
    monkeypatch.setattr(bootstrap, "_load", values.__getitem__)

    with pytest.raises(TypeError, match="does not resolve to a class|Module subclass"):
        bootstrap.main(
            "decl:Declaration",
            "impl:Implementation",
            9,
            base64.b64encode(pickle.dumps({})).decode("ascii"),
        )


def test_typer_cli_parses_bootstrap_options(monkeypatch: pytest.MonkeyPatch) -> None:
    class NotImplementation:
        pass

    monkeypatch.setattr(
        bootstrap,
        "_load",
        {"decl:Declaration": Declaration, "impl:Implementation": NotImplementation}.__getitem__,
    )
    app = typer.Typer()
    app.command()(bootstrap.main)
    result = CliRunner().invoke(
        app,
        [
            "--declaration",
            "decl:Declaration",
            "--implementation",
            "impl:Implementation",
            "--handshake-fd",
            "9",
            "--kwargs",
            base64.b64encode(pickle.dumps({})).decode("ascii"),
        ],
    )

    assert result.exit_code != 0
    assert "Missing option" not in result.output


def test_main_valid_reference_serves_and_signals_ready(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeRPC:
        def serve_module_rpc(self, module: object, name: str) -> None:
            self.served = (module, name)

    class FakeModule:
        rpc = FakeRPC()

        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class FakeExternal:
        pass

    class FakeDeclaration(FakeExternal):
        pass

    class FakeImplementation(FakeDeclaration, FakeModule):
        pass

    monkeypatch.setattr(bootstrap, "ExternalPythonModule", FakeExternal)
    monkeypatch.setattr(bootstrap, "Module", FakeModule)
    monkeypatch.setattr(
        bootstrap,
        "_load",
        {
            "decl:Declaration": FakeDeclaration,
            "impl:Implementation": FakeImplementation,
        }.__getitem__,
    )
    writes: list[tuple[int, bytes]] = []
    monkeypatch.setattr(
        bootstrap.os, "write", lambda fd, data: writes.append((fd, data)) or len(data)
    )
    monkeypatch.setattr(bootstrap.os, "close", lambda _: None)
    monkeypatch.setattr(bootstrap.time, "sleep", lambda _: (_ for _ in ()).throw(KeyboardInterrupt))

    with pytest.raises(KeyboardInterrupt):
        bootstrap.main(
            "decl:Declaration",
            "impl:Implementation",
            9,
            base64.b64encode(pickle.dumps({"value": 7})).decode("ascii"),
        )

    assert writes == [(9, b"READY\n")]
    assert FakeImplementation.rpc.served[1] == "FakeDeclaration"
    assert FakeImplementation.rpc.served[0].kwargs == {"value": 7}


def test_real_child_reports_startup_failure_and_closes_handshake() -> None:
    read_fd, write_fd = os.pipe()
    reference = f"{__name__}:Declaration"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "dimos.core.external_python_bootstrap",
            "--declaration",
            reference,
            "--implementation",
            f"{__name__}:BrokenImplementation",
            "--handshake-fd",
            str(write_fd),
            "--kwargs",
            base64.b64encode(pickle.dumps({})).decode("ascii"),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        pass_fds=(write_fd,),
    )
    os.close(write_fd)
    handshake = os.read(read_fd, 4096)
    os.close(read_fd)
    stdout, stderr = process.communicate(timeout=10)

    assert process.returncode != 0
    assert handshake.startswith(b"ERROR RuntimeError:")
    assert b"constructor failed deliberately" in stderr
    assert len(stdout) < 64 * 1024
