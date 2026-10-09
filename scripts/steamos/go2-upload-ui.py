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

"""Upload window for the Go2 handheld: every recording, its status, live progress, one button.

Runs on the venv's Python (Tk ships with it) inside the dimos container; the window shows on
the handheld's desktop. Drives `dimos data upload <memory.db>` one recording at a time and
follows its plain progress lines (printed when stdout is not a terminal). A finished upload is
marked with an `.uploaded` file next to the store, the same convention go2-upload.sh and its
timer use, and the two share a lock so they never run at once.
"""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk

RECORDINGS = Path.home() / "dimos" / "recordings"
APP_DIR = Path.home() / "dimos" / "dimensional-applications"
DIMOS = APP_DIR / ".venv" / "bin" / "dimos"
LOCK = Path.home() / ".go2-upload.lock"
PHASES = {"compress": "compressing", "upload": "uploading"}


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1000:
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1000
    return f"{n:.1f} TB"


def online() -> bool:
    try:
        socket.create_connection(("api.dimensional.org", 443), timeout=3).close()
        return True
    except OSError:
        return False


def running_run_id() -> str:
    try:
        out = subprocess.run([str(DIMOS), "status"], capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    for line in out.stdout.splitlines():
        if "Run ID:" in line:
            return line.split()[-1]
    return ""


class Recording:
    def __init__(self, folder: Path) -> None:
        self.folder = folder
        self.db = folder / "memory.db"
        self.size = self.db.stat().st_size if self.db.exists() else 0
        self.uploaded = (folder / ".uploaded").exists()
        self.status = "uploaded" if self.uploaded else "pending"
        self.done = 0
        self.total = 0
        self.note = ""

    @property
    def name(self) -> str:
        # 20261008-053211-63f4-unitree-go2-gamepad-cockpit -> 2026-10-08 05:32:11  gamepad-cockpit
        # The run id's stamp is the device's local clock at start, 24-hour.
        stamp, _, rest = self.folder.name.partition("-")
        hms = rest[:6]
        bp = rest.split("-", 2)[-1].replace("unitree-go2-", "") if "-" in rest else rest
        when = f"{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]} {hms[:2]}:{hms[2:4]}:{hms[4:6]}"
        return f"{when}  {bp}"


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Go2 recordings")
        self.geometry("980x560")
        self.minsize(720, 400)
        self.configure(bg="#14161b")
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background="#14161b", foreground="#e6e9ee", fieldbackground="#1c1f26")
        style.configure("Treeview", rowheight=34, font=("DejaVu Sans", 13), borderwidth=0)
        style.configure("Treeview.Heading", font=("DejaVu Sans", 12, "bold"), background="#1c1f26")
        style.map("Treeview", background=[("selected", "#2a3140")])
        style.configure("TProgressbar", troughcolor="#1c1f26", background="#00aaff", thickness=18)
        style.configure("Big.TButton", font=("DejaVu Sans", 14, "bold"), padding=(18, 10))
        style.configure("TLabel", font=("DejaVu Sans", 12))

        top = ttk.Frame(self, padding=(16, 14, 16, 6))
        top.pack(fill="x")
        self.net = ttk.Label(top, text="checking network...")
        self.net.pack(side="left")
        self.button = ttk.Button(top, text="Upload all", style="Big.TButton", command=self.start)
        self.button.pack(side="right")

        cols = ("recording", "size", "status", "progress")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", selectmode="none")
        for c, w, anchor in (
            ("recording", 420, "w"),
            ("size", 110, "e"),
            ("status", 200, "w"),
            ("progress", 160, "w"),
        ):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor=anchor, stretch=(c == "recording"))
        self.tree.tag_configure("uploaded", foreground="#8a93a3")
        self.tree.tag_configure("active", foreground="#00aaff")
        self.tree.tag_configure("failed", foreground="#eb4848")
        self.tree.pack(fill="both", expand=True, padx=16)

        bottom = ttk.Frame(self, padding=(16, 10, 16, 14))
        bottom.pack(fill="x")
        self.bar = ttk.Progressbar(bottom, mode="determinate", maximum=1000)
        self.bar.pack(fill="x")
        self.line = ttk.Label(bottom, text="")
        self.line.pack(anchor="w", pady=(6, 0))

        self.recs: list[Recording] = []
        self.events: queue.Queue[tuple[str, Recording, object]] = queue.Queue()
        self.worker: threading.Thread | None = None
        self.refresh()
        self.after(200, self.pump)
        self.after(500, self.check_net)
        self.after(1500, self.start)  # one tap: the icon opens the window and it goes

    def refresh(self) -> None:
        self.recs = sorted(
            (Recording(p) for p in RECORDINGS.glob("*/") if (p / "memory.db").exists()),
            key=lambda r: r.folder.name,
            reverse=True,
        )
        self.tree.delete(*self.tree.get_children())
        for r in self.recs:
            self.tree.insert("", "end", iid=str(r.folder), values=self.row(r), tags=(r.status,))
        pending = [r for r in self.recs if not r.uploaded]
        self.line.config(
            text=f"{len(self.recs)} recordings, {len(pending)} to upload, "
            f"{human(sum(r.size for r in pending))} pending"
        )

    def row(self, r: Recording) -> tuple[str, str, str, str]:
        pct = f"{100 * r.done / r.total:3.0f}%" if r.total else ""
        if r.status in PHASES.values() and r.total:
            pct += f"  {human(r.done)} / {human(r.total)}"
        return (r.name, human(r.size), r.status + (f"  {r.note}" if r.note else ""), pct)

    def update_row(self, r: Recording) -> None:
        self.tree.item(str(r.folder), values=self.row(r), tags=(r.status,))

    def check_net(self) -> None:
        def probe() -> None:
            self.events.put(("net", self.recs[0] if self.recs else Recording(RECORDINGS), online()))

        threading.Thread(target=probe, daemon=True).start()
        self.after(10000, self.check_net)

    def start(self) -> None:
        if self.worker and self.worker.is_alive():
            return
        pending = [r for r in self.recs if not r.uploaded]
        if not pending:
            self.line.config(text="Everything is uploaded.")
            return
        self.button.state(["disabled"])
        self.worker = threading.Thread(target=self.run_all, args=(pending,), daemon=True)
        self.worker.start()

    def run_all(self, pending: list[Recording]) -> None:
        lock = open(LOCK, "w")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.events.put(("busy", pending[0], None))
            return
        try:
            if not online():
                self.events.put(("offline", pending[0], None))
                return
            live = running_run_id()
            for r in pending:
                if live and live in r.folder.name:
                    r.note = "recording now"
                    self.events.put(("row", r, None))
                    continue
                self.upload_one(r)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()
            self.events.put(("done", pending[0], None))

    def upload_one(self, r: Recording) -> None:
        r.status, r.done, r.total, r.note = "starting", 0, 0, ""
        self.events.put(("row", r, None))
        proc = subprocess.Popen(
            [str(DIMOS), "data", "upload", str(r.db)],
            cwd=APP_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
        )
        tail: list[str] = []
        assert proc.stdout is not None
        for line in proc.stdout:
            line = line.rstrip("\n")
            if line.startswith("progress\t"):
                _, phase, done, total, _name = line.split("\t", 4)
                r.status = PHASES.get(phase, phase)
                r.done, r.total = int(done), int(total)
                self.events.put(("row", r, None))
            elif line.strip():
                tail.append(line.strip())
                if "console preview" in line:
                    r.note = "preview sent"
        code = proc.wait()
        if code == 0:
            (r.folder / ".uploaded").touch()
            r.uploaded, r.status, r.done, r.total = True, "uploaded", r.size, r.size
            r.note = "(already there)" if any("already uploaded" in t for t in tail) else r.note
        else:
            r.status, r.note = "failed", (tail[-1][:60] if tail else "see ~/go2-app.log")
        self.events.put(("row", r, None))

    def pump(self) -> None:
        try:
            while True:
                kind, r, payload = self.events.get_nowait()
                if kind == "row":
                    self.update_row(r)
                    if r.status in PHASES.values() and r.total:
                        self.bar["value"] = 1000 * r.done / r.total
                        self.line.config(
                            text=f"{r.status} {r.name}: {human(r.done)} of {human(r.total)}"
                        )
                elif kind == "net":
                    self.net.config(
                        text="online" if payload else "offline, uploads wait for a network",
                        foreground="#38c878" if payload else "#eb4848",
                    )
                elif kind == "offline":
                    self.line.config(text="No internet here. Recordings are kept and upload later.")
                    self.button.state(["!disabled"])
                elif kind == "busy":
                    self.line.config(text="An upload is already running in the background.")
                    self.button.state(["!disabled"])
                elif kind == "done":
                    self.bar["value"] = 1000
                    left = [x for x in self.recs if not x.uploaded]
                    self.line.config(
                        text="All uploaded."
                        if not left
                        else f"{len(left)} not uploaded, see status column."
                    )
                    self.button.state(["!disabled"])
        except queue.Empty:
            pass
        self.after(150, self.pump)


if __name__ == "__main__":
    if not RECORDINGS.exists():
        RECORDINGS.mkdir(parents=True)
    App().mainloop()
    sys.exit(0)
