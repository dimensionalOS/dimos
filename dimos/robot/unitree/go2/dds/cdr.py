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

"""Generic little-endian CDR (XCDR1) decoder driven by a field spec.

A spec class declares ``__cdr_fields__`` — an ordered list of ``(name, type)``.
``type`` is a primitive code (``"u8"``, ``"f32"`` …), ``"string"``, a nested spec
class, or ``("array", elem, n)`` / ``("seq", elem)``. ``decode(buf, Cls)`` walks
the body with CDR alignment and returns a populated instance.

This replaces per-message hand-rolled decoders: the wire layout lives in the
spec, not in code. The same spec also generates the IDL we embed into mcaps.
"""

from __future__ import annotations

import struct
from typing import Any

import numpy as np

# code -> (struct char, byte size, numpy dtype)
_PRIM: dict[str, tuple[str, int, str]] = {
    "i8": ("b", 1, "<i1"),
    "u8": ("B", 1, "<u1"),
    "i16": ("h", 2, "<i2"),
    "u16": ("H", 2, "<u2"),
    "i32": ("i", 4, "<i4"),
    "u32": ("I", 4, "<u4"),
    "i64": ("q", 8, "<i8"),
    "u64": ("Q", 8, "<u8"),
    "f32": ("f", 4, "<f4"),
    "f64": ("d", 8, "<f8"),
    "bool": ("?", 1, "<?"),
}


def _align_of(t: Any) -> int:
    """CDR alignment (bytes) of a field type."""
    if isinstance(t, str):
        if t == "string":
            return 4  # u32 length prefix
        return _PRIM[t][1]
    if isinstance(t, tuple):
        if t[0] == "array":
            return _align_of(t[1])
        if t[0] == "seq":
            return 4  # u32 length prefix
    if isinstance(t, type):
        return _struct_align(t)
    raise TypeError(f"unknown field type {t!r}")


def _struct_align(cls: Any) -> int:
    a = getattr(cls, "__cdr_align__", None)
    if a is None:
        a = max((_align_of(t) for _, t in cls.__cdr_fields__), default=1)
        cls.__cdr_align__ = a
    return a


class Cursor:
    """Body-relative cursor; offset 0 is just after the 4-byte encapsulation header."""

    __slots__ = ("b", "p")

    def __init__(self, b: bytes) -> None:
        self.b = b
        self.p = 4  # skip CDR encapsulation header

    def align(self, n: int) -> None:
        m = (self.p - 4) % n
        if m:
            self.p += n - m

    def prim(self, code: str) -> Any:
        ch, sz, _ = _PRIM[code]
        self.align(sz)
        v = struct.unpack_from("<" + ch, self.b, self.p)[0]
        self.p += sz
        return v

    def prim_array(self, code: str, n: int) -> np.ndarray:
        _, sz, dt = _PRIM[code]
        self.align(sz)
        a = np.frombuffer(self.b, dt, n, self.p).copy()
        self.p += sz * n
        return a

    def string(self) -> str:
        n = self.prim("u32")
        v = self.b[self.p : self.p + max(0, n - 1)].decode("ascii", "replace")
        self.p += n
        return v


def _read(cur: Cursor, t: Any) -> Any:
    if isinstance(t, str):
        return cur.string() if t == "string" else cur.prim(t)
    if isinstance(t, tuple):
        kind, elem = t[0], t[1]
        n = t[2] if kind == "array" else cur.prim("u32")
        if isinstance(elem, str) and elem in _PRIM:
            return cur.prim_array(elem, n)
        return [_read(cur, elem) for _ in range(n)]
    if isinstance(t, type):
        cur.align(_struct_align(t))
        return _read_struct(cur, t)
    raise TypeError(f"unknown field type {t!r}")


def _read_struct(cur: Cursor, cls: Any) -> Any:
    return cls(**{name: _read(cur, t) for name, t in cls.__cdr_fields__})


def decode(buf: bytes, cls: Any) -> tuple[Any, int]:
    """Decode ``buf`` as a CDR ``cls``. Returns ``(instance, end_offset)``.

    ``end_offset`` should equal ``len(buf)`` for a fixed-layout message — the
    cheapest correctness check against a real recording.
    """
    cur = Cursor(buf)
    cur.align(_struct_align(cls))
    return _read_struct(cur, cls), cur.p
