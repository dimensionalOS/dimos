#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
Detect CUDA (nvcc → nvidia-smi) and MERGE a CUDA-specific dependency block into
[project.optional-dependencies].cuda in pyproject.toml — without extra blanks.

- Pure stdlib, works on Python 3.10+ (no tomllib needed).
- Supports CUDA 12.1 / 12.4 / 12.6 / 12.9.
- Order-preserving, de-duplicated merge.
- Rewrites ONLY the 'cuda = [ ... ]' array inside the
  [project.optional-dependencies] section; nothing else.

Usage:
  python cuda_inject.py [path/to/pyproject.toml]
"""

from __future__ import annotations
import pathlib, re, subprocess, sys
from typing import List, Optional

# ---------- CUDA detection ----------------------------------------------------
_NVCC_RE = re.compile(r"release\s+(\d+\.\d+)")
_SMI_RE = re.compile(r"CUDA Version:\s*(\d+\.\d+)")


def detect_cuda_version() -> Optional[str]:
    # 1) nvcc
    try:
        out = subprocess.check_output(
            ["nvcc", "--version"], encoding="utf-8", stderr=subprocess.STDOUT
        )
        m = _NVCC_RE.search(out)
        if m:
            return m.group(1)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    # 2) nvidia-smi
    try:
        out = subprocess.check_output(["nvidia-smi"], encoding="utf-8", stderr=subprocess.STDOUT)
        m = _SMI_RE.search(out)
        if m:
            return m.group(1)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    return None


# ---------- CUDA version → deps mapping (xformers per version) ----------------
# If you later need different xformers pins per minor, just change the lines below.
CUDA_REQUIREMENTS = {
    "12.1": [
        "torch==2.1.0+cu121",
        "torchvision==0.16.0+cu121",
        "xformers==0.0.32.post2",
        "nvidia-cublas-cu12==12.1.3.1",
        "nvidia-cuda-cupti-cu12==12.1.105",
        "nvidia-cuda-nvrtc-cu12==12.1.105",
        "nvidia-cuda-runtime-cu12==12.1.105",
        "nvidia-cudnn-cu12==8.9.2.26",
        "nvidia-cufft-cu12==11.0.2.54",
        "nvidia-curand-cu12==10.3.2.106",
        "nvidia-cusolver-cu12==11.4.5.107",
        "nvidia-cusparse-cu12==12.0.2.54",
        "nvidia-nccl-cu12==2.18.3",
        "nvidia-nvtx-cu12==12.1.105",
        "cupy-cuda12x==13.6.0",
        "cucim==23.10.0",
    ],
    "12.4": [
        "torch==2.8.0+cu124",
        "torchvision==0.23.0+cu124",
        "xformers==0.0.32.post2",
        "nvidia-cublas-cu12==12.4.5.4",
        "nvidia-cuda-cupti-cu12==12.4.105",
        "nvidia-cuda-nvrtc-cu12==12.4.105",
        "nvidia-cuda-runtime-cu12==12.4.105",
        "nvidia-cudnn-cu12==9.0.0.312",
        "nvidia-cufft-cu12==11.2.1.3",
        "nvidia-curand-cu12==10.3.4.107",
        "nvidia-cusolver-cu12==11.5.1.3",
        "nvidia-cusparse-cu12==12.3.0.3",
        "nvidia-nccl-cu12==2.21.5",
        "nvidia-nvtx-cu12==12.4.105",
        "cupy-cuda12x==13.6.0",
        "cucim==23.10.0",
    ],
    "12.6": [
        "torch==2.8.0+cu126",
        "torchvision==0.23.0+cu126",
        "xformers==0.0.32.post2",
        "nvidia-cublas-cu12==12.6.2.1",
        "nvidia-cuda-cupti-cu12==12.6.68",
        "nvidia-cuda-nvrtc-cu12==12.6.68",
        "nvidia-cuda-runtime-cu12==12.6.68",
        "nvidia-cudnn-cu12==9.5.1.17",
        "nvidia-cufft-cu12==11.3.0.3",
        "nvidia-curand-cu12==10.3.7.3",
        "nvidia-cusolver-cu12==11.6.0.3",
        "nvidia-cusparse-cu12==12.4.1.3",
        "nvidia-nccl-cu12==2.25.1",
        "nvidia-nvtx-cu12==12.6.68",
        "cupy-cuda12x==13.6.0",
        "cucim==23.10.0",
    ],
    "12.9": [
        "torch==2.8.0+cu129",
        "torchvision==0.23.0+cu129",
        "xformers==0.0.32.post2",
        "nvidia-cublas-cu12==12.9.1.4",
        "nvidia-cuda-cupti-cu12==12.9.79",
        "nvidia-cuda-nvrtc-cu12==12.9.86",
        "nvidia-cuda-runtime-cu12==12.9.79",
        "nvidia-cudnn-cu12==9.10.2.21",
        "nvidia-cufft-cu12==11.4.1.4",
        "nvidia-cufile-cu12==1.14.1.1",
        "nvidia-curand-cu12==10.3.10.19",
        "nvidia-cusolver-cu12==11.7.5.82",
        "nvidia-cusparse-cu12==12.5.10.65",
        "nvidia-cusparselt-cu12==0.7.1",
        "nvidia-nccl-cu12==2.27.3",
        "nvidia-nvjitlink-cu12==12.9.86",
        "nvidia-nvtx-cu12==12.9.79",
        "cupy-cuda12x==13.6.0",
        "cucim==23.10.0",
    ],
}

# ---------- Locators for the exact place to edit (no EOF appends) -------------
SEC_PROJ_OPT = re.compile(r"(?ms)^(\s*\[project\.optional-dependencies\]\s*\n)(.*?)(?=^\s*\[|\Z)")
CUDA_KEYLINE = re.compile(r"(?m)^\s*cuda\s*=\s*\[")

# Quoted-string finder inside a TOML array (simple, suits our deps)
QUOTED = re.compile(r'"([^"\\]*(?:\\.[^"\\]*)*)"')


def _find_cuda_array_span(section_body: str) -> tuple[int, int] | None:
    """Return (start, end) indices of the cuda=[...] array text within section_body,
    or None if not present. Robustly finds the matching closing bracket, ignoring strings."""
    m = CUDA_KEYLINE.search(section_body)
    if not m:
        return None
    i = m.start()
    # Find first '[' after '='
    eq = section_body.find("=", i)
    if eq == -1:
        return None
    lb = section_body.find("[", eq)
    if lb == -1:
        return None
    # Scan forward to matching ']'
    depth, j = 0, lb
    in_str = False
    escape = False
    while j < len(section_body):
        ch = section_body[j]
        if in_str:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_str = False
        else:
            if ch == '"':
                in_str = True
            elif ch == "[":
                depth += 1
            elif ch == "]":
                depth -= 1
                if depth == 0:
                    # include trailing whitespace/newlines
                    k = j + 1
                    while k < len(section_body) and section_body[k] in " \t\r\n":
                        k += 1
                    return (i, k)
        j += 1
    return None


def _parse_existing_items(array_text: str) -> List[str]:
    """Extract quoted items from a TOML array text."""
    return [m.group(1) for m in QUOTED.finditer(array_text)]


def _order_preserving_merge(existing: List[str], additions: List[str]) -> List[str]:
    seen = set()
    merged = []
    for x in existing + additions:
        if x not in seen:
            merged.append(x)
            seen.add(x)
    return merged


def _format_cuda_array(values: List[str], indent: str = "") -> str:
    # No blank lines; one item per line, trailing commas
    lines = [f"{indent}cuda = ["]
    for v in values:
        lines.append(f'{indent}    "{v}",')
    lines.append(f"{indent}]")
    return "\n".join(lines)


def inject_cuda(pyproject_path: str = "pyproject.toml") -> int:
    ver = detect_cuda_version()
    if not ver:
        sys.stderr.write("cuda_inject: No CUDA detected (nvcc/nvidia-smi not found or no GPU).\n")
        return 1
    mm = ".".join(ver.split(".")[:2])
    deps = CUDA_REQUIREMENTS.get(mm)
    if not deps:
        sys.stderr.write(f"cuda_inject: No dependency set for CUDA {mm}.\n")
        return 1

    p = pathlib.Path(pyproject_path)
    if not p.exists():
        sys.stderr.write(f"cuda_inject: File not found: {pyproject_path}\n")
        return 2

    text = p.read_text(encoding="utf-8")

    sec = SEC_PROJ_OPT.search(text)
    if not sec:
        # Create the whole section fresh at EOF
        sep = "" if text.endswith("\n") else "\n"
        new_body = _format_cuda_array(deps)
        new_text = text + f"{sep}\n[project.optional-dependencies]\n{new_body}\n"
        p.write_text(new_text, encoding="utf-8")
        print(f"cuda_inject: Wrote new [project.optional-dependencies].cuda for CUDA {mm} ({ver})")
        return 0

    header, body = sec.group(1), sec.group(2)
    span = _find_cuda_array_span(body)

    if span:
        s, e = span
        array_text = body[s:e]
        # Keep the original indentation of the 'cuda' key
        indent_match = re.match(r"^(\s*)", array_text)
        indent = indent_match.group(1) if indent_match else ""
        existing = _parse_existing_items(array_text)
        merged = _order_preserving_merge(existing, deps)
        replacement = _format_cuda_array(merged, indent=indent)
        new_body = body[:s] + replacement + "\n" + body[e:]
    else:
        # No cuda array yet → append inside the section
        glue = "" if body.endswith("\n") else "\n"
        new_body = body + glue + _format_cuda_array(deps) + "\n"

    start, end = sec.span()
    new_text = text[:start] + header + new_body + text[end:]
    p.write_text(new_text, encoding="utf-8")
    print(f"cuda_inject: Merged CUDA {mm} ({ver}) deps into [project.optional-dependencies].cuda")
    return 0


if __name__ == "__main__":
    sys.exit(inject_cuda(sys.argv[1]) if len(sys.argv) > 1 else inject_cuda())
