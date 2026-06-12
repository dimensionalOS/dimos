from __future__ import annotations

import os
import platform
import shlex
import socket
import subprocess
import sys
from pathlib import Path
from xml.sax.saxutils import escape

_DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=1"

_LABEL = "com.dimensional.dimwizard"
_PLIST_PATH = Path.home() / "Library" / "LaunchAgents" / f"{_LABEL}.plist"
_SYSTEMD_PATH = Path.home() / ".config" / "systemd" / "user" / "dimwizard.service"
_LOG_PATH = Path.home() / "Library" / "Logs" / "dimwizard.log"


def _find_executable() -> list[str]:
    return [sys.executable, "-m", "dimwizard"]


def is_installed() -> bool:
    if platform.system() == "Darwin":
        return _PLIST_PATH.exists()
    if platform.system() == "Linux":
        return _SYSTEMD_PATH.exists()
    return False


def is_running() -> bool:
    if platform.system() == "Darwin":
        result = subprocess.run(
            ["launchctl", "list", _LABEL],
            capture_output=True,
            text=True,
        )
        return result.returncode == 0 and '"PID"' in result.stdout
    if platform.system() == "Linux":
        result = subprocess.run(
            ["systemctl", "--user", "is-active", "dimwizard"],
            capture_output=True,
            text=True,
        )
        return result.stdout.strip() == "active"
    return False


def install() -> bool:
    if platform.system() == "Darwin":
        return _install_mac()
    if platform.system() == "Linux":
        return _install_linux()
    print(f"  Unsupported platform: {platform.system()}")
    return False


def uninstall() -> None:
    if platform.system() == "Darwin":
        _uninstall_mac()
    elif platform.system() == "Linux":
        _uninstall_linux()



def _install_mac() -> bool:
    _PLIST_PATH.parent.mkdir(parents=True, exist_ok=True)
    _LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
    lcm_url = os.environ.get("LCM_DEFAULT_URL", _DEFAULT_LCM_URL)

    executable = _find_executable()
    plist_args = "\n".join(f"        <string>{escape(a)}</string>" for a in executable)
    log_path = escape(str(_LOG_PATH))

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
    <key>EnvironmentVariables</key>
    <dict>
        <key>DIMENSIONAL_ROBOT_NAME</key>
        <string>{escape(robot_name)}</string>
        <key>LCM_DEFAULT_URL</key>
        <string>{escape(lcm_url)}</string>
    </dict>
    <key>KeepAlive</key>
    <dict>
        <key>SuccessfulExit</key>
        <false/>
    </dict>
    <key>RunAtLoad</key>
    <true/>
    <key>StandardOutPath</key>
    <string>{log_path}</string>
    <key>StandardErrorPath</key>
    <string>{log_path}</string>
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
        _PLIST_PATH.unlink(missing_ok=True)
        return False
    print(f"  dimwizard installed — logs at {_LOG_PATH}")
    return True


def _uninstall_mac() -> None:
    if not _PLIST_PATH.exists():
        print("dimwizard is not installed.")
        return
    subprocess.run(["launchctl", "unload", "-w", str(_PLIST_PATH)], capture_output=True)
    _PLIST_PATH.unlink()
    print("  dimwizard removed.")



def _install_linux() -> bool:
    _SYSTEMD_PATH.parent.mkdir(parents=True, exist_ok=True)

    robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
    lcm_url = os.environ.get("LCM_DEFAULT_URL", _DEFAULT_LCM_URL)

    # systemd quoted-string values must have " escaped as \"
    robot_name_esc = robot_name.replace('"', '\\"')
    lcm_url_esc = lcm_url.replace('"', '\\"')

    exec_start = " ".join(shlex.quote(a) for a in _find_executable())

    unit = f"""\
[Unit]
Description=DimWizard — Dimensional robot network beacon
After=network.target

[Service]
Type=simple
ExecStart={exec_start}
Restart=on-failure
RestartSec=5
Environment=PYTHONUNBUFFERED=1
Environment="DIMENSIONAL_ROBOT_NAME={robot_name_esc}"
Environment="LCM_DEFAULT_URL={lcm_url_esc}"

[Install]
WantedBy=default.target
"""
    _SYSTEMD_PATH.write_text(unit)

    try:
        subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
        subprocess.run(["systemctl", "--user", "enable", "--now", "dimwizard"], check=True)
        print("  dimwizard installed.")
        return True
    except FileNotFoundError:
        print(f"  systemctl not found — start manually: {exec_start}")
        _SYSTEMD_PATH.unlink(missing_ok=True)
        return False
    except subprocess.CalledProcessError as e:
        print(f"  Failed to enable service: {e}")
        _SYSTEMD_PATH.unlink(missing_ok=True)
        return False


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
