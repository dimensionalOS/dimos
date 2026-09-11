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

from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.cli.shell import _format_description, _shell_namespace
from dimos.core.introspection.module.info import ModuleInfo, ParamInfo, RpcInfo
from dimos.porcelain.dimos import Dimos


def test_shell_rejects_non_interactive_execution(mocker):
    mocker.patch("dimos.cli.shell._is_interactive_terminal", return_value=False)
    connect = mocker.patch("dimos.cli.shell.Dimos.connect")

    result = CliRunner().invoke(main, ["shell"])

    assert result.exit_code == 1
    assert "requires an interactive terminal" in result.output
    assert "Dimos Python interface" in result.output
    connect.assert_not_called()


def test_shell_starts_ipython_with_debug_namespace_and_disconnects(mocker):
    app = mocker.Mock(spec=Dimos)
    mocker.patch("dimos.cli.shell._is_interactive_terminal", return_value=True)
    connect = mocker.patch("dimos.cli.shell.Dimos.connect", return_value=app)
    mocker.patch("dimos.cli.shell.get_most_recent", return_value=None)
    start_ipython = mocker.patch("IPython.start_ipython")

    result = CliRunner().invoke(main, ["shell"])

    assert result.exit_code == 0, result.output
    connect.assert_called_once_with()
    assert start_ipython.call_args.kwargs["argv"] == ["--no-banner", "--no-tip"]
    namespace = start_ipython.call_args.kwargs["user_ns"]
    assert set(namespace) == {"app", "guide", "modules", "rpcs", "describe"}
    assert namespace["app"] is app
    assert callable(namespace["guide"])
    assert callable(namespace["modules"])
    assert callable(namespace["rpcs"])
    assert callable(namespace["describe"])
    assert "▇▇▇▇▇▇╗" in result.output
    assert "RPC shell" in result.output
    assert "Quick start" in result.output
    assert 'describe("ModuleName.method")' in result.output
    assert "app.ModuleName.method?" in result.output
    assert "unregistered coordinator" in result.output
    assert "RPC calls execute immediately" in result.output
    app.stop.assert_called_once_with()


def test_shell_guide_reprints_quick_start_guide(mocker, capsys):
    app = mocker.Mock(spec=Dimos)

    result = _shell_namespace(app)["guide"]()

    assert result is None
    output = capsys.readouterr().out
    assert "Quick start" in output
    assert "guide()" in output
    assert "modules()" in output
    assert 'rpcs("ModuleName")' in output
    assert 'describe("ModuleName.method")' in output
    assert "app.ModuleName.method(...)" in output
    assert "app.ModuleName.method?" in output


def test_shell_modules_prints_compact_table(mocker, capsys):
    app = mocker.Mock(spec=Dimos)
    app.list_modules.return_value = [
        ModuleInfo(
            name="ExampleModule",
            instance_name="robot0/example",
            class_name="ExampleModule",
            rpcs=[RpcInfo(name="ping")],
        )
    ]

    result = _shell_namespace(app)["modules"]()

    assert result is None
    output = capsys.readouterr().out
    assert "Instance" in output
    assert "Class" in output
    assert "RPCs" in output
    assert "robot0/example" in output
    assert "ExampleModule" in output


def test_shell_rpcs_prints_signatures_and_docstring_summaries(mocker, capsys):
    app = mocker.Mock(spec=Dimos)
    app.list_rpcs.return_value = [
        RpcInfo(
            name="echo",
            module_name="ExampleModule",
            params=[ParamInfo("message", "str")],
            return_type="str",
            documentation="Echo a message.\n\nMore details.",
        )
    ]

    result = _shell_namespace(app)["rpcs"]("ExampleModule")

    assert result is None
    app.list_rpcs.assert_called_once_with("ExampleModule")
    output = capsys.readouterr().out
    assert "ExampleModule.echo" in output
    assert "echo(message: str) -> str" in output
    assert "Echo a message." in output
    assert "More details." not in output


def test_shell_describe_prints_readable_rpc_details(mocker, capsys):
    rpc = RpcInfo(
        name="echo",
        module_name="ExampleModule",
        params=[ParamInfo("message", "str")],
        return_type="str",
        documentation="Echo a message back to the caller.",
    )
    app = mocker.Mock(spec=Dimos)
    app.describe.return_value = rpc

    result = _shell_namespace(app)["describe"]("ExampleModule.echo")

    assert result is None
    assert capsys.readouterr().out == (
        "RPC: ExampleModule.echo\n"
        "Signature: echo(message: str) -> str\n"
        "\n"
        "Documentation:\n"
        "Echo a message back to the caller.\n"
    )


def test_format_module_description_includes_class_docs_and_rpcs():
    info = ModuleInfo(
        name="ExampleModule",
        instance_name="robot0/example",
        class_name="ExampleModule",
        qualified_path="example.module.ExampleModule",
        documentation="An example module.",
        rpcs=[RpcInfo(name="ping", return_type="str")],
    )

    description = _format_description(info)

    assert "Module: robot0/example" in description
    assert "Class: example.module.ExampleModule" in description
    assert "An example module." in description
    assert "ping() -> str" in description


def test_shell_reports_connection_failure(mocker):
    mocker.patch("dimos.cli.shell._is_interactive_terminal", return_value=True)
    connect = mocker.patch(
        "dimos.cli.shell.Dimos.connect",
        side_effect=RuntimeError("No running DimOS coordinator found"),
    )
    start_ipython = mocker.patch("IPython.start_ipython")

    result = CliRunner().invoke(main, ["shell"])

    assert result.exit_code == 1
    assert "No running DimOS coordinator found" in result.output
    connect.assert_called_once_with()
    start_ipython.assert_not_called()


def test_shell_disconnects_when_ipython_reports_connection_loss(mocker):
    app = mocker.Mock(spec=Dimos)
    mocker.patch("dimos.cli.shell._is_interactive_terminal", return_value=True)
    connect = mocker.patch("dimos.cli.shell.Dimos.connect", return_value=app)
    mocker.patch("dimos.cli.shell.get_most_recent", return_value=None)
    mocker.patch(
        "IPython.start_ipython",
        side_effect=RuntimeError("coordinator connection lost"),
    )

    result = CliRunner().invoke(main, ["shell"])

    assert result.exit_code == 1
    assert isinstance(result.exception, RuntimeError)
    assert str(result.exception) == "coordinator connection lost"
    connect.assert_called_once_with()
    app.stop.assert_called_once_with()
