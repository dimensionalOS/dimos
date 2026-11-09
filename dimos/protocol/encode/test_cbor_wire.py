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

"""The Rust SDK speaks the portable subset of this codec, byte for byte.

The vectors below are written as fixtures that the Rust suite decodes and
re-encodes (``native/rust/dimos-module/src/rpc/cbor.rs``). Locally the test
rewrites the fixtures. In CI it checks they are current.
"""

from __future__ import annotations

import difflib
import importlib
import inspect
import json
import os
from pathlib import Path
import subprocess
from typing import Any

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.encode.cbor import encode

_ROOT = Path(__file__).parents[3]
_FIXTURES = _ROOT / "native" / "rust" / "dimos-module" / "tests" / "fixtures"

# Named the same in the Rust test's expected() table.
_VECTORS: dict[str, Any] = {
    "null": None,
    "true": True,
    "false": False,
    "int_zero": 0,
    "int_small": 1,
    "int_negative": -1,
    "int_large": 2**40,
    "int_min": -(2**63),
    "int_max": 2**63 - 1,
    "float_one": 1.0,
    "float_negative_zero": -0.0,
    "float_subnormal": 5e-324,
    "float_large": 1e300,
    "float_nan": float("nan"),
    "float_inf": float("inf"),
    "float_neg_inf": float("-inf"),
    "str": "héllo",
    "str_empty": "",
    "bytes": b"\x00\x01\xff",
    "bytes_empty": b"",
    "list": [1, "a", None],
    "list_empty": [],
    "tuple": (1, 2.5),
    "tuple_empty": (),
    "dict_str_keys": {"a": 1, "b": [2]},
    "dict_empty": {},
    "dict_primitive_keys": {1: "a", None: 2, b"k": True, 2.5: None},
    "dict_tuple_keys": {(1, 2): "t"},
    "nested": ([{"k": (1, b"x")}], {"m": {"n": [None]}}),
    "rpc_arguments": ((1.0, "goal"), {"timeout": 2.5}),
    "msg_wrapped": PoseStamped(
        position=[1.0, 2.0, 3.0], orientation=[0.0, 0.0, 0.0, 1.0], ts=1.5, frame_id="map"
    ),
    "msg_lcm_only": GoalStatus(
        goal_id=GoalID(stamp=Time(sec=7, nsec=8), id="g1"), status=1, text="active"
    ),
}


def _wrapped_message_names() -> list[str]:
    """LCM type names that a class under dimos.msgs encodes and decodes."""
    names: set[str] = set()
    for path in sorted((_ROOT / "dimos" / "msgs").glob("*/*.py")):
        if path.name.startswith(("test_", "__")):
            continue
        module = importlib.import_module(f"dimos.msgs.{path.parent.name}.{path.stem}")
        for _, cls in inspect.getmembers(module, inspect.isclass):
            if cls.__module__ != module.__name__:
                continue
            msg_name = getattr(cls, "msg_name", None)
            if isinstance(msg_name, str) and callable(getattr(cls, "lcm_encode", None)):
                names.add(msg_name)
    return sorted(names)


def _check_golden(path: Path, content: str) -> None:
    if "CI" in os.environ:
        current = path.read_text() if path.exists() else ""
        if current != content:
            diff = "".join(
                difflib.unified_diff(
                    current.splitlines(keepends=True),
                    content.splitlines(keepends=True),
                    fromfile=f"{path.name} (current)",
                    tofile=f"{path.name} (generated)",
                )
            )
            pytest.fail(
                f"{path.name} is out of date. Run `pytest {Path(__file__).relative_to(_ROOT)}` "
                f"locally to update.\n\n{diff}"
            )
        return
    path.write_text(content)
    status = subprocess.run(
        ["git", "status", "--porcelain", "--", str(path)],
        cwd=_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    if status.strip():
        pytest.fail(f"{path.name} was updated and has uncommitted changes. Please commit it.")


def test_vectors_are_current() -> None:
    vectors = {name: encode(value).hex() for name, value in _VECTORS.items()}
    _check_golden(_FIXTURES / "rpc_cbor.json", json.dumps(vectors, indent=2, sort_keys=True) + "\n")


def test_wrapped_message_names_are_current() -> None:
    names = _wrapped_message_names()
    assert "geometry_msgs.PoseStamped" in names
    _check_golden(_FIXTURES / "msg_wrappers.json", json.dumps(names, indent=2) + "\n")
