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

from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).with_name("docstring_style.py")


@pytest.mark.parametrize(
    "docstring",
    [
        "Read a value.\n\nParameters\n----------\nvalue : int\n    Input value.",
        "Read a value.\n\nReturns\n-------\nint\n    The value.",
        "Read a value.\n\n:param value: Input value.",
        "Read a value.\n\n:param int value: Input value.",
        "Read a value.\n\n:returns: The value.",
        "Read a value.\n\n:rtype: int",
        "Read a value.\n\n:raises ValueError: Invalid value.",
        "Store a value.\n\n:ivar value: Stored value.",
        "Read a value.\n\n@param value: Input value.",
        "Read a value.\n\nExamples:\n    >>> print(1)\n\nReturns\n-------\nint",
    ],
)
def test_rejects_other_docstring_sections(tmp_path, docstring):
    source = tmp_path / "example.py"
    source.write_text(f"def example(value):\n    {docstring!r}\n    return value\n")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)], capture_output=True, text=True
    )
    assert result.returncode == 1
    assert "DOCSTYLE" in result.stdout
    assert "use Google sections" in result.stdout


@pytest.mark.parametrize(
    "docstring",
    [
        "Read a value.",
        "Read a value: preserve its type.\n\nUse :class:`Thing` for values.",
        "Read a value.\n\nArgs:\n    value: Input value.\n\nReturns:\n    The value.",
        "Store a value.\n\nAttributes:\n    value: Stored value.",
        "Read a value.\n\nExamples:\n    Parameters\n    ----------\n    :param value: example text",
        "Read a value.\n\n```python\nParameters\n----------\n:param value: example\n```",
        "Read a value.\n\n~~~text\nReturns\n-------\n~~~",
        "Read a value.\n\n````text\n```\nParameters\n----------\n````",
        "Read a value.\n\nExample text::\n\n    Parameters\n    ----------\n    :param value: example",
        "Read a value.\n\n.. code-block:: python\n\n    :param value: example",
        "Read a value.\n\nExample code:\n\n    Parameters\n    ----------\n    :param value: example",
        "Read a value.\n\n>>> text = ':param value: example'",
    ],
)
def test_accepts_google_prose_and_examples(tmp_path, docstring):
    source = tmp_path / "example.py"
    source.write_text(f"def example(value):\n    {docstring!r}\n    return value\n")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)], capture_output=True, text=True
    )
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_ignores_comments_strings_and_missing_docstrings_or_types(tmp_path):
    source = tmp_path / "example.py"
    source.write_text(
        "# :param value: ordinary comment\n"
        "EXAMPLE = 'Parameters\\n----------\\n:param value: string data'\n"
        "def example(value):\n    return value\n"
    )
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)], capture_output=True, text=True
    )
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_checks_nested_async_class_and_module_docstrings(tmp_path):
    source = tmp_path / "example.py"
    source.write_text(
        "'''A module.\n\n:returns: module result\n'''\n"
        "class Example:\n    '''A class.\n\n    :param value: input\n    '''\n"
        "    async def run(self):\n        '''Run.\n\n        :rtype: int\n        '''\n"
    )
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)], capture_output=True, text=True
    )
    assert result.returncode == 1
    assert result.stdout.count("DOCSTYLE") == 3
