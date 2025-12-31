#!/usr/bin/env python3
from __future__ import annotations

import platform
import re
from pathlib import Path
from typing import Dict, Literal, TypedDict

from .dax import command_exists, run_quiet


class ToolResult(TypedDict, total=False):
    name: str
    exists: bool
    version: str
    raw: str
    note: str


OSName = Literal["macos", "windows", "debianBased", "archBased", "fedora", "unknownLinux", "unknown"]

OS_NAMES: tuple[OSName, ...] = (
    "macos",
    "windows",
    "debianBased",
    "archBased",
    "fedora",
    "unknownLinux",
    "unknown",
)


def _extract_digits_dots(text: str) -> str | None:
    m = re.search(r"\b(\d+(?:\.\d+)+)\b", text)
    return m.group(1) if m else None


def _run_first_line(cmd: list[str] | str) -> tuple[int, str, str]:
    res = run_quiet(cmd)
    combined = f"{res.stdout}\n{res.stderr}".strip()
    line = ""
    for l in combined.splitlines():
        if l.strip():
            line = l.strip()
            break
    return res.code, line, combined


def _get_version_from_command(
    name: str,
    cmd: list[str] | str,
    *,
    allow_raw_if_no_digits: bool = False,
    note: str | None = None,
) -> tuple[str, ToolResult]:
    code, line, combined = _run_first_line(cmd)
    if code != 0 and not line:
        return name, {"name": name, "exists": True, "note": "Command exists but version query failed."}

    ver = _extract_digits_dots(line) or _extract_digits_dots(combined)
    if ver:
        return name, {"name": name, "exists": True, "version": ver, "raw": line, "note": note or None}

    return (
        name,
        {
            "name": name,
            "exists": True,
            "raw": line or (combined.splitlines()[0].strip() if combined else ""),
            "note": note
            if allow_raw_if_no_digits
            else "No digit-dot version found.",
        },
    )


def _detect_os_name() -> OSName:
    system = platform.system().lower()
    if system == "darwin":
        return "macos"
    if system == "windows":
        return "windows"
    if system == "linux":
        has_apt = command_exists("apt-get")
        has_pacman = command_exists("pacman")
        has_dnf = command_exists("dnf")
        if has_apt:
            return "debianBased"
        if has_pacman:
            return "archBased"
        if has_dnf:
            return "fedora"
        return "unknownLinux"
    return "unknown"


def _detect_os_details(os_name: OSName) -> dict[str, str]:
    if os_name == "macos":
        code, line, _ = _run_first_line(["sw_vers", "-productVersion"])
        return {"version": _extract_digits_dots(line) or line, "raw": line, "note": "macOS version"}
    if os_name == "windows":
        code, line, _ = _run_first_line(["cmd", "/c", "ver"])
        return {"version": _extract_digits_dots(line) or "", "raw": line}
    if os_name in {"debianBased", "archBased", "fedora", "unknownLinux"}:
        try:
            text = Path("/etc/os-release").read_text()
            version_match = re.search(r"^VERSION_ID=(.*)$", text, re.MULTILINE)
            pretty_match = re.search(r"^PRETTY_NAME=(.*)$", text, re.MULTILINE)
            version_id = version_match.group(1) if version_match else ""
            pretty = pretty_match.group(1) if pretty_match else ""
            version_clean = version_id.strip('"')
            pretty_clean = pretty.strip('"')
            return {
                "version": _extract_digits_dots(version_clean) or version_clean or "",
                "raw": pretty_clean or None,
            }
        except Exception:
            _code, line, _combined = _run_first_line(["uname", "-sr"])
            return {"raw": line}
    return {}


def get_tool_check_results() -> Dict[str, ToolResult]:
    results: Dict[str, ToolResult] = {}

    existence = {cmd: command_exists(cmd) for cmd in ["git", "nix", "docker", "python3", "python", "pip3", "pip", "nvcc", "nvidia-smi"]}

    tasks: list[tuple[str, ToolResult]] = []

    if existence["git"]:
        tasks.append(_get_version_from_command("git", ["git", "--version"], allow_raw_if_no_digits=True))
    else:
        results["git"] = {"name": "git", "exists": False}

    if existence["nix"]:
        tasks.append(_get_version_from_command("nix", ["nix", "--version"], allow_raw_if_no_digits=True))
    else:
        results["nix"] = {"name": "nix", "exists": False}

    if existence["docker"]:
        tasks.append(_get_version_from_command("docker", ["docker", "--version"], allow_raw_if_no_digits=True))
    else:
        results["docker"] = {"name": "docker", "exists": False}

    if existence["git"]:
        name, val = _get_version_from_command("git_lfs", ["git", "lfs", "version"], allow_raw_if_no_digits=True)
        if val.get("exists") and not val.get("version"):
            note = val.get("note", "")
            val["note"] = (note + " " if note else "") + "If this says 'git: lfs is not a git command', Git LFS isn't installed."
        tasks.append((name, val))
    else:
        results["git_lfs"] = {
            "name": "git_lfs",
            "exists": False,
            "note": "git not found, so git lfs cannot be checked.",
        }

    python_cmd = "python3" if existence["python3"] else ("python" if existence["python"] else None)
    if python_cmd:
        tasks.append(
            _get_version_from_command(
                "python",
                [python_cmd, "--version"],
                allow_raw_if_no_digits=True,
                note=f"From {python_cmd}",
            )
        )
    else:
        results["python"] = {"name": "python", "exists": False}

    if python_cmd:
        name, val = _get_version_from_command(
            "pip",
            [python_cmd, "-m", "pip", "--version"],
            allow_raw_if_no_digits=True,
            note=f"From {python_cmd} -m pip",
        )
        if val.get("exists") and "no module named pip" in (val.get("raw", "") or "").lower():
            val = {
                "name": "pip",
                "exists": False,
                "raw": val.get("raw", ""),
                "note": "Python found, but pip module not installed.",
            }
        tasks.append((name, val))
    elif existence["pip3"]:
        tasks.append(_get_version_from_command("pip", ["pip3", "--version"], allow_raw_if_no_digits=True, note="From pip3"))
    elif existence["pip"]:
        tasks.append(_get_version_from_command("pip", ["pip", "--version"], allow_raw_if_no_digits=True, note="From pip"))
    else:
        results["pip"] = {"name": "pip", "exists": False}

    if existence["nvcc"]:
        code, _, combined = _run_first_line(["nvcc", "--version"])
        version = None
        m = re.search(r"release\s+(\d+(?:\.\d+)+)", combined, re.IGNORECASE)
        if m:
            version = m.group(1)
        else:
            version = _extract_digits_dots(combined)
        first_line = combined.strip().splitlines()[0] if combined else ""
        tasks.append(
            (
                "cuda",
                {"name": "cuda", "exists": True, "version": version, "raw": first_line, "note": "From nvcc"},
            )
        )
    elif existence["nvidia-smi"]:
        res = run_quiet(["nvidia-smi"])
        combined = f"{res.stdout}\n{res.stderr}".strip()
        m = re.search(r"CUDA Version:\s*([0-9]+(?:\.[0-9]+)+)", combined, re.IGNORECASE)
        ver = m.group(1) if m else _extract_digits_dots(combined)
        first_line = combined.splitlines()[0].strip() if combined else ""
        tasks.append(
            (
                "cuda",
                {
                    "name": "cuda",
                    "exists": res.code == 0,
                    "version": ver,
                    "raw": first_line,
                    "note": "From nvidia-smi",
                },
            )
        )
    else:
        results["cuda"] = {"name": "cuda", "exists": False, "note": "Neither nvcc nor nvidia-smi found."}

    os_name = _detect_os_name()
    details = _detect_os_details(os_name)
    tasks.append(("os", {"name": os_name, "exists": True, **details}))

    for key, value in tasks:
        results[key] = value

    return results


__all__ = ["ToolResult", "OS_NAMES", "OSName", "get_tool_check_results"]
