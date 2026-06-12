from __future__ import annotations

import platform
import subprocess
import sys
from pathlib import Path

_LABEL = "com.dimensional.dimwizard"
_PLIST_PATH = Path.home() / "Library" / "LaunchAgents" / f"{_LABEL}.plist"
_SYSTEMD_PATH = Path.home() / ".config" / "systemd" / "user" / "dimwizard.service"
_LOG_PATH = Path.home() / "Library" / "Logs" / "dimwizard.log"
_EXECUTABLE = [sys.executable, "-m", "dimwizard"]


def install() -> None:
    if platform.system() == "Darwin":
        _install_mac()
    elif platform.system() == "Linux":
        _install_linux()
    else:
        print(f"  Unsupported platform: {platform.system()}")


def uninstall() -> None:
    if platform.system() == "Darwin":
        _uninstall_mac()
    elif platform.system() == "Linux":
        _uninstall_linux()


# ── macOS ─────────────────────────────────────────────────────────────────────

def _install_mac() -> None:
    _PLIST_PATH.parent.mkdir(parents=True, exist_ok=True)
    _LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    plist_args = "\n".join(f"        <string>{a}</string>" for a in _EXECUTABLE)

    plist = f"""\
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>{_LABEL}</string>
    <key>ProgramArguments</key>
    <array>
{plist_args}
    </array>
    <key>KeepAlive</key>
    <true/>
    <key>RunAtLoad</key>
    <true/>
    <key>StandardOutPath</key>
    <string>{_LOG_PATH}</string>
    <key>StandardErrorPath</key>
    <string>{_LOG_PATH}</string>
</dict>
</plist>
"""
    _PLIST_PATH.write_text(plist)

    subprocess.run(["launchctl", "unload", str(_PLIST_PATH)], capture_output=True)
    result = subprocess.run(
        ["launchctl", "load", "-w", str(_PLIST_PATH)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"  Warning: launchctl load failed: {result.stderr.strip()}")
    else:
        print(f"  dimwizard installed — logs at {_LOG_PATH}")


def _uninstall_mac() -> None:
    if not _PLIST_PATH.exists():
        print("dimwizard is not installed.")
        return
    subprocess.run(["launchctl", "unload", "-w", str(_PLIST_PATH)], capture_output=True)
    _PLIST_PATH.unlink()
    print("  dimwizard removed.")


# ── Linux ─────────────────────────────────────────────────────────────────────

def _install_linux() -> None:
    _SYSTEMD_PATH.parent.mkdir(parents=True, exist_ok=True)

    exec_start = " ".join(_EXECUTABLE)

    unit = f"""\
[Unit]
Description=DimWizard — Dimensional robot network beacon
After=network.target

[Service]
Type=simple
ExecStart={exec_start}
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
"""
    _SYSTEMD_PATH.write_text(unit)

    try:
        subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
        subprocess.run(["systemctl", "--user", "enable", "--now", "dimwizard"], check=True)
        print("  dimwizard installed.")
    except FileNotFoundError:
        print(f"  systemctl not found — start manually: {exec_start}")
    except subprocess.CalledProcessError as e:
        print(f"  Failed to enable service: {e}")


def _uninstall_linux() -> None:
    if not _SYSTEMD_PATH.exists():
        print("dimwizard is not installed.")
        return
    try:
        subprocess.run(["systemctl", "--user", "disable", "--now", "dimwizard"], check=True)
    except subprocess.CalledProcessError:
        pass
    _SYSTEMD_PATH.unlink()
    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True)
    print("  dimwizard removed.")
