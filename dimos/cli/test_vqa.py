# Copyright 2026 Dimensional Inc.

from typer.testing import CliRunner

from dimos.cli import vqa


def test_vqa_generation_cli_has_no_explicit_query_or_model_options() -> None:
    output = CliRunner().invoke(vqa.app, ["generate", "--help"]).output

    assert "--query" not in output
    assert "--propose-questions" not in output
    assert "--question-model" not in output
    assert "--oracle-model" not in output
