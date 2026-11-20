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

"""Extract the ONNX policies embedded in the vendor rl_deploy binary (they are vendor IP, so they are not committed).

rl_deploy (/opt/robot/share/rl_deploy/bin/rl_deploy on the M20's AOS board) links each policy as a byte array named
`__policy_<name>_` with its size in `__policy_<name>__len`; this reads both from the ELF symbol table.

    scp the binary off the robot, then:
    python -m dimos.simulation.m20_policy_sim.extract_policies rl_deploy ~/.cache/dimos/m20_policies
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import struct

# md5 of the rl_deploy v1.5.8 binary these docs and configs were decoded from
KNOWN_BINARY_MD5 = "b16070a4bf2abb52a0ce33e863a4df2e"


def elf_symbols(data: bytes) -> dict[str, tuple[int, int]]:
    """Name -> (file offset, size) for every defined symbol of a little-endian ELF64 file."""
    if data[:4] != b"\x7fELF" or data[4] != 2 or data[5] != 1:
        raise ValueError("not a little-endian ELF64 file")
    (shoff,) = struct.unpack_from("<Q", data, 0x28)
    shentsize, shnum = struct.unpack_from("<HH", data, 0x3A)
    sections = [
        struct.unpack_from("<IIQQQQIIQQ", data, shoff + i * shentsize) for i in range(shnum)
    ]
    symbols: dict[str, tuple[int, int]] = {}
    for _name, kind, _flags, _addr, offset, size, link, _info, _align, entsize in sections:
        if kind not in (2, 11):  # SHT_SYMTAB, SHT_DYNSYM
            continue
        strtab_offset = sections[link][4]
        for k in range(size // entsize):
            name_off, _info, _other, shndx, value, sym_size = struct.unpack_from(
                "<IBBHQQ", data, offset + k * entsize
            )
            if shndx == 0 or shndx >= shnum:
                continue
            end = data.index(b"\0", strtab_offset + name_off)
            name = data[strtab_offset + name_off : end].decode(errors="replace")
            section = sections[shndx]
            symbols[name] = (value - section[3] + section[4], sym_size)
    return symbols


def extract(binary: Path, out_dir: Path) -> list[Path]:
    data = binary.read_bytes()
    digest = hashlib.md5(data).hexdigest()
    if digest != KNOWN_BINARY_MD5:
        print(f"warning: {binary} md5 {digest} is not the decoded v1.5.8 ({KNOWN_BINARY_MD5})")
    symbols = elf_symbols(data)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for name, (offset, _size) in sorted(symbols.items()):
        if not (name.startswith("__policy_") and name.endswith("_") and not name.endswith("__len")):
            continue
        length_symbol = symbols.get(name + "_len")
        if length_symbol is None:
            continue
        (length,) = struct.unpack_from("<Q", data, length_symbol[0])
        blob = data[offset : offset + length]
        if b"pytorch" not in blob[:32]:
            print(f"skip {name}: not an ONNX model")
            continue
        path = out_dir / f"{name[len('__policy_') : -1]}.onnx"
        path.write_bytes(blob)
        written.append(path)
        print(f"{path} {length} bytes md5 {hashlib.md5(blob).hexdigest()}")
    return written


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("binary", type=Path, help="the rl_deploy executable")
    parser.add_argument("out_dir", type=Path, help="where to write <name>.onnx")
    args = parser.parse_args()
    if not extract(args.binary, args.out_dir):
        raise SystemExit("no __policy_* symbols found")


if __name__ == "__main__":
    main()
