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

"""Finding, validating and observing the dimos a live eval case drives."""

from collections.abc import Iterator, Sequence
from contextlib import contextmanager
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

from dimos.core.global_config import global_config
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT


def default_mcp_url() -> str:
    return f"http://localhost:{global_config.mcp_port}/mcp"


def validate_blueprints(names: Sequence[str]) -> None:
    """Validate the registry names passed to ``dimos run``."""
    # Resolving blueprints loads their optional robot dependencies.
    from dimos.robot.get_all_blueprints import get_by_name

    for name in names:
        get_by_name(name)


def rerun_url() -> str:
    return f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"


def _end(proc: subprocess.Popen[bytes], sig: int = signal.SIGINT, wait_s: float = 15.0) -> None:
    if proc.poll() is None:
        proc.send_signal(sig)
        try:
            proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


@contextmanager
def rrd_recorder(path: Path, url: str) -> Iterator[None]:
    """Stream the viewer's data to ``path`` with the rerun CLI; no display needed."""
    proc = subprocess.Popen(
        ["rerun", "--save", str(path), url],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        yield
    finally:
        _end(proc)


def _window_region(display: str, screen: str, wait_s: float = 90.0) -> list[str]:
    """``-video_size WxH -i :N+X,Y`` for the viewer's window, or the whole screen if not found."""
    sw, sh = (int(v) for v in screen.split("x"))
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        tree = subprocess.run(
            ["xwininfo", "-root", "-tree", "-display", display],
            capture_output=True, text=True, check=False,
        ).stdout  # fmt: skip
        for line in tree.splitlines():
            m = re.search(r"(\d+)x(\d+)\+(\d+)\+(\d+)\s+\+(\d+)\+(\d+)$", line)
            if m and ("rerun" in line.lower() or "viewer" in line.lower()):
                x, y = int(m.group(5)), int(m.group(6))
                w, h = min(int(m.group(1)), sw - x) // 2 * 2, min(int(m.group(2)), sh - y) // 2 * 2
                return ["-video_size", f"{w}x{h}", "-i", f"{display}+{x},{y}"]
        time.sleep(0.5)
    Path("/tmp/dimos-eval-xwininfo.txt").write_text(tree)  # why the window was not found
    return ["-video_size", screen, "-i", display]


def _free_display() -> str:
    n = 90
    while Path(f"/tmp/.X{n}-lock").exists() or Path(f"/tmp/.X11-unix/X{n}").exists():
        n += 1
    return f":{n}"


def _park_pointer(display: str, x: int, y: int) -> None:
    """Move the pointer off the viewer so no hover tooltip lands in the recording."""
    import ctypes

    x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XWarpPointer.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        *[ctypes.c_int] * 6,
    ]
    x11.XFlush.argtypes = x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    d = x11.XOpenDisplay(display.encode())
    if d:
        x11.XWarpPointer(d, 0, x11.XDefaultRootWindow(d), 0, 0, 0, 0, x, y)
        x11.XFlush(d)
        x11.XCloseDisplay(d)


@contextmanager
def screen_capture(path: Path, url: str, size: str = "1920x1080", fps: int = 15) -> Iterator[None]:
    """The viewer connected to ``url`` on a virtual display, captured to ``path`` as H.264."""
    display = _free_display()
    null = subprocess.DEVNULL
    w, h = (int(v) for v in size.split("x"))
    screen = f"{w + 512}x{h + 256}"  # room for the window, which is not placed at 0,0 without a WM
    xvfb = subprocess.Popen(
        ["Xvfb", display, "-screen", "0", f"{screen}x24", "-nolisten", "tcp"],
        stdin=null, stdout=null, stderr=null,
    )  # fmt: skip
    deadline = time.monotonic() + 10.0
    while not Path(f"/tmp/.X11-unix/X{display[1:]}").exists() and time.monotonic() < deadline:
        time.sleep(0.1)
    env = {**os.environ, "DISPLAY": display}
    beside = Path(sys.executable).with_name("dimos-viewer")
    viewer = subprocess.Popen(
        [
            str(beside) if beside.exists() else shutil.which("dimos-viewer") or "rerun",
            "--connect", url,
            "--window-size", size,
            "--hide-welcome-screen",
            "--expect-data-soon",
            "--memory-limit", "2GB",  # the default is a share of host RAM: 30 GB on a big box
        ],
        env=env, stdin=null, stdout=null, stderr=null,
    )  # fmt: skip
    region = _window_region(display, screen)
    _park_pointer(display, w + 511, h + 255)
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-loglevel", "error", "-y",
            "-f", "x11grab", "-draw_mouse", "0", "-framerate", str(fps),
            *region,
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", str(path),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=path.with_suffix(".ffmpeg.log").open("w"),
        env=env,
    )  # fmt: skip
    try:
        yield
    finally:
        _end(ffmpeg)  # SIGINT finalizes the container
        _end(viewer, signal.SIGTERM)
        _end(xvfb, signal.SIGTERM)
