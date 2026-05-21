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

"""Structural parity between PGOCpp, PGORust, and the LoopClosure Spec.

nav_stack picks one or the other via the `loop_closure` parameter, which
is only safe if both modules declare the same In/Out streams with the same
types. This test reads class annotations only — no binaries spin up — so it
runs in <1s.
"""

from __future__ import annotations

import typing

import pytest

from dimos.navigation.nav_stack.modules.pgo_cpp.pgo_cpp import PGOCpp
from dimos.navigation.nav_stack.modules.pgo_rust.pgo_rust import PGORust
from dimos.navigation.nav_stack.specs import LoopClosure


def _stream_annotations(cls: type) -> dict[str, type]:
    """Collect class-level stream type annotations from `cls` and its bases.

    NativeModule and LoopClosure declare streams as class-body annotations
    (e.g. `corrected_odometry: Out[Odometry]`).  `typing.get_type_hints`
    resolves string-form annotations and walks the MRO.
    """
    hints = typing.get_type_hints(cls)
    # Filter to only the I/O-stream entries; drop everything else (Config
    # subclass references, internal scalars, lifecycle hooks).
    keep_origins = {"In", "Out", "Optional"}
    streams: dict[str, type] = {}
    for name, hint in hints.items():
        origin = getattr(hint, "__name__", "") or getattr(
            getattr(hint, "__class__", None), "__name__", ""
        )
        # In[T] and Out[T] are generic; their repr shape lives in the stream module.
        type_repr = repr(hint)
        if "In[" in type_repr or "Out[" in type_repr:
            streams[name] = hint
        elif origin in keep_origins:
            streams[name] = hint
    return streams


def _signature_of(streams: dict[str, type]) -> set[str]:
    """Reduce a stream-annotation dict to a comparable signature.

    Each entry becomes "name: repr(type)" so we catch both rename and type
    changes (e.g. `Out[PoseStamped]` vs `Out[Odometry]`).
    """
    return {f"{name}: {hint!r}" for name, hint in streams.items()}


CPP_STREAMS = _stream_annotations(PGOCpp)
RUST_STREAMS = _stream_annotations(PGORust)
SPEC_STREAMS = _stream_annotations(LoopClosure)


def test_pgo_rust_and_pgo_cpp_publish_the_same_streams() -> None:
    """Both backends must expose identical In/Out stream annotations so
    nav_stack can swap them under the `loop_closure` parameter."""
    cpp_sig = _signature_of(CPP_STREAMS)
    rust_sig = _signature_of(RUST_STREAMS)
    assert cpp_sig == rust_sig, (
        f"PGOCpp and PGORust diverge:\n"
        f"  only in cpp:  {sorted(cpp_sig - rust_sig)}\n"
        f"  only in rust: {sorted(rust_sig - cpp_sig)}"
    )


def test_both_backends_implement_loop_closure_spec() -> None:
    """Both must cover every stream the LoopClosure Spec Protocol declares."""
    spec_sig = _signature_of(SPEC_STREAMS)
    cpp_sig = _signature_of(CPP_STREAMS)
    rust_sig = _signature_of(RUST_STREAMS)
    missing_in_cpp = spec_sig - cpp_sig
    missing_in_rust = spec_sig - rust_sig
    assert not missing_in_cpp, f"PGOCpp missing LoopClosure streams: {sorted(missing_in_cpp)}"
    assert not missing_in_rust, f"PGORust missing LoopClosure streams: {sorted(missing_in_rust)}"


@pytest.mark.parametrize(
    "stream_name",
    [
        "registered_scan",
        "odometry",
        "corrected_odometry",
        "global_map",
        "pose_graph",
        "loop_closure_event",
    ],
)
def test_each_required_stream_present_on_both(stream_name: str) -> None:
    """Spot-check each named stream individually for clearer failure output
    than the all-at-once signature comparison above."""
    assert stream_name in CPP_STREAMS, f"PGOCpp missing stream: {stream_name}"
    assert stream_name in RUST_STREAMS, f"PGORust missing stream: {stream_name}"
    assert repr(CPP_STREAMS[stream_name]) == repr(RUST_STREAMS[stream_name]), (
        f"stream {stream_name!r} type diverges: "
        f"cpp={CPP_STREAMS[stream_name]!r}, rust={RUST_STREAMS[stream_name]!r}"
    )
