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

"""Optional native MuJoCo window displaying snapshots from the running world."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import pickle
import select
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any

from dimos.core.stream import Transport
from dimos.sim2.messages import StateFrame

if TYPE_CHECKING:
    import mujoco.viewer

    from dimos.sim2.sensors.reader import WorldReader

_DISPLAY_PERIOD = 1.0 / 30.0
_STOP_TIMEOUT = 2.0


class NativeMujocoViewer:
    """Own a separate viewer process; no graphics or simulation work runs here.

    The supplied transport is copied into the child using its normal worker
    serialization. Only the child subscribes; the publisher remains untouched.
    """

    def __init__(self, description: dict[str, Any], transport: Transport[StateFrame]) -> None:
        self._description = description
        self._transport = transport
        self._process: subprocess.Popen[bytes] | None = None
        self._directory: tempfile.TemporaryDirectory[str] | None = None
        self._error: str | None = None
        self._exitcode: int | None = None
        self._lock = threading.RLock()

    def start(self, timeout: float = 30.0) -> None:
        """Wait for a received snapshot and an open window, or raise with cleanup."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return
            self.stop()
            self._error = None
            self._exitcode = None
            executable = sys.executable
            if sys.platform == "darwin":
                local = Path(sys.executable).with_name("mjpython")
                executable = str(local) if local.is_file() else shutil.which("mjpython") or ""
                if not executable:
                    raise FileNotFoundError("Native MuJoCo viewer requires mjpython on macOS")
            self._directory = tempfile.TemporaryDirectory(prefix="dimos-mujoco-viewer-")
            directory = Path(self._directory.name)
            try:
                # Private, local bootstrap: transport __reduce__ reconstructs its
                # connection in the child instead of copying a live session.
                bootstrap = directory / "bootstrap.pkl"
                bootstrap.write_bytes(pickle.dumps((self._description, self._transport)))
                self._process = subprocess.Popen(
                    [executable, "-m", "dimos.sim2.world.viewer", str(bootstrap)],
                    stdin=subprocess.PIPE,
                )
                deadline = time.monotonic() + timeout
                while not (directory / "ready.json").is_file():
                    code = self._process.poll()
                    if code is not None:
                        error = directory / "error.txt"
                        detail = error.read_text() if error.is_file() else f"exit code {code}"
                        raise RuntimeError(f"Native MuJoCo viewer failed to open: {detail}")
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            "Native MuJoCo viewer did not receive a snapshot and open "
                            f"within {timeout:g}s"
                        )
                    time.sleep(0.05)
            except Exception as error:
                self._error = str(error)
                self.stop()
                raise

    def status(self) -> dict[str, Any]:
        with self._lock:
            code = self._process.poll() if self._process is not None else self._exitcode
            result: dict[str, Any] = {
                "running": self._process is not None and code is None,
                "pid": self._process.pid if self._process is not None else None,
                "exitcode": code,
                "error": self._error,
            }
            if self._directory is not None:
                directory = Path(self._directory.name)
                ready = directory / "ready.json"
                if ready.is_file():
                    result.update(json.loads(ready.read_text()))
                error = directory / "error.txt"
                if error.is_file():
                    result["error"] = error.read_text()
                elif code not in (None, 0):
                    result["error"] = f"Native MuJoCo viewer exited with code {code}"
            return result

    def stop(self) -> None:
        """Close the control pipe, then bound shutdown even if the GUI has stalled."""
        with self._lock:
            if self._process is not None:
                if self._process.stdin is not None:
                    self._process.stdin.close()
                try:
                    self._process.wait(timeout=_STOP_TIMEOUT)
                except subprocess.TimeoutExpired:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=_STOP_TIMEOUT)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                        self._process.wait()
                self._exitcode = self._process.returncode
                self._process = None
            if self._directory is not None:
                error = Path(self._directory.name) / "error.txt"
                if error.is_file():
                    self._error = error.read_text()
                self._directory.cleanup()
                self._directory = None


def _parent_closed(timeout: float) -> bool:
    # No commands travel on stdin: readability means EOF after the owning
    # process closes its pipe (including when the owner itself exits).
    return bool(select.select([sys.stdin], [], [], timeout)[0])


def _configure_camera(viewer: mujoco.viewer.Handle, reader: WorldReader, robots: list[str]) -> None:
    import mujoco  # Optional native dependency, used only inside the viewer process.

    for robot in robots:
        root = reader.root_body(robot)
        if root is None:
            continue
        body = mujoco.mj_name2id(
            reader.model, mujoco.mjtObj.mjOBJ_BODY, reader.model_prefix(robot) + root
        )
        if body >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = body
            viewer.cam.lookat[:] = reader.data.xpos[body]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -20.0
            break


def _view(description: dict[str, Any], transport: Transport[StateFrame], directory: Path) -> None:
    # Keep optional MuJoCo/robosuite dependencies out of the generic module's import path.
    import mujoco.viewer

    from dimos.sim2.sensors.reader import WorldReader

    reader = WorldReader(description)
    unsubscribe = None
    try:
        unsubscribe = transport.subscribe(reader.push)
        # Never display the MJB's default pose while waiting for the live world.
        while not reader.update():
            if _parent_closed(_DISPLAY_PERIOD):
                return
        reopen_deadline = 0.0
        while not _parent_closed(0.0):
            model = reader.model
            try:
                handle = mujoco.viewer.launch_passive(
                    model, reader.data, show_left_ui=True, show_right_ui=False
                )
            except RuntimeError as error:
                # mjpython releases its sole UI slot asynchronously after
                # close(). Retry only that documented teardown race.
                if (
                    str(error) != "another MuJoCo viewer is already open"
                    or time.monotonic() >= reopen_deadline
                ):
                    raise
                if _parent_closed(_DISPLAY_PERIOD):
                    return
                continue
            options = {
                name: copy.deepcopy(getattr(model.opt, name))
                for name in dir(model.opt)
                if not name.startswith("_")
            }
            with handle as viewer:
                with viewer.lock():
                    viewer.opt.geomgroup[:] = 1
                    _configure_camera(viewer, reader, list(description.get("root_bodies", {})))
                viewer.set_texts(
                    (None, None, "DimOS: read-only snapshots", "Control and pause in DimOS")
                )
                ready = directory / "ready.tmp"
                ready.write_text(
                    json.dumps(
                        {"initial_sequence": reader.sequence, "initial_episode": reader.episode}
                    )
                )
                ready.replace(directory / "ready.json")
                while viewer.is_running():
                    before = time.monotonic()
                    with viewer.lock():
                        # The inspection UI exposes physics settings too. Do
                        # not let local edits change reconstructed contacts.
                        for name, value in options.items():
                            setattr(model.opt, name, value)
                        reader.update()
                        if reader.model is not model:
                            break
                        viewer.perturb.active = 0
                    # WorldReader already ran forward dynamics for contacts;
                    # state_only=True would perform that work a second time.
                    viewer.sync()
                    if _parent_closed(max(0.0, _DISPLAY_PERIOD - (time.monotonic() - before))):
                        return
            if reader.model is model:
                # Closing the window is intentional; a model reload reopens it.
                return
            reopen_deadline = time.monotonic() + _STOP_TIMEOUT
    finally:
        if unsubscribe is not None:
            unsubscribe()
        transport.stop()


def main() -> None:
    """Child entry point; invoke through NativeMujocoViewer to own its lifecycle."""
    bootstrap = Path(sys.argv[1])
    try:
        description, transport = pickle.loads(bootstrap.read_bytes())
        _view(description, transport, bootstrap.parent)
    except Exception:
        (bootstrap.parent / "error.txt").write_text(traceback.format_exc())
        raise


if __name__ == "__main__":
    main()
