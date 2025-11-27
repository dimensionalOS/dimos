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

"""MkDocs hooks for pre-building marimo notebooks.

See docs/development.md "Embedding Marimo Notebooks" for why we use this approach
(dimos is not available in Pyodide/WASM, so mkdocs-marimo's native embedding won't work).

Why we kill the marimo export process instead of letting it exit gracefully:

The tutorial notebooks start a Dask cluster which registers a SIGTERM signal handler.
When dimos.stop() is called, the handler runs close_all() and then sys.exit(0).
However, marimo's runtime catches SystemExit exceptions, preventing the process from
actually exiting. The process hangs indefinitely waiting for... something in marimo.

This is a marimo-specific issue - the same notebook code exits cleanly when run as
a regular Python script. Since we can't change marimo's exception handling, we poll
for the output file to be written (which happens early, before shutdown) and then
kill the process once the file is ready.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

# Marimo notebooks to export as HTML with outputs
MARIMO_NOTEBOOKS = [
    {
        "source": "docs/tutorials/skill_basics/tutorial.py",
        "output": "docs/tutorials/skill_basics/tutorial_rendered.html",
    },
    {
        "source": "docs/tutorials/skill_with_agent/tutorial.py",
        "output": "docs/tutorials/skill_with_agent/tutorial_rendered.html",
    },
    {
        "source": "docs/tutorials/multi_agent/tutorial.py",
        "output": "docs/tutorials/multi_agent/tutorial_rendered.html",
    },
]


def _export_notebook(source: Path, output: Path, timeout: int = 90) -> bool:
    """Export a marimo notebook, killing the process once the file is ready.

    The notebooks use Dask which hangs on shutdown, but the HTML is generated
    within the first few seconds. This function polls for the output file and
    kills the process early once it's ready, rather than waiting for the full timeout.

    Returns True if the export succeeded, False otherwise.
    """
    # Record initial mtime if file exists (to detect when it's updated)
    initial_mtime = output.stat().st_mtime if output.exists() else 0

    proc = subprocess.Popen(
        ["marimo", "export", "html", str(source), "-o", str(output), "--force"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        # Create new session so marimo + its Dask workers share a process group.
        # This lets us kill them all at once with os.killpg(). See _kill_process_tree().
        start_new_session=True,
    )

    start = time.time()
    poll_interval = 0.5  # Check every 500ms
    min_file_size = 1000  # HTML should be at least 1KB

    try:
        while time.time() - start < timeout:
            # Check if process finished naturally
            if proc.poll() is not None:
                if proc.returncode == 0:
                    return True
                else:
                    stderr = proc.stderr.read().decode() if proc.stderr else ""
                    print(f"  Export failed: {stderr[:500]}")
                    return False

            # Check if output file is ready (exists, updated, and has content)
            if output.exists():
                current_mtime = output.stat().st_mtime
                current_size = output.stat().st_size
                if current_mtime > initial_mtime and current_size > min_file_size:
                    # File is ready, kill the hanging process
                    _kill_process_tree(proc)
                    print(f"  Generated ({current_size // 1024}KB, killed hanging process)")
                    return True

            time.sleep(poll_interval)

        # Timeout reached
        _kill_process_tree(proc)
        if output.exists() and output.stat().st_size > min_file_size:
            print("  Timeout but file was generated")
            return True
        else:
            print("  Timeout and file was not generated")
            return False

    except Exception as e:
        _kill_process_tree(proc)
        print(f"  Error: {e}")
        return False


def _kill_process_tree(proc: subprocess.Popen):
    """Kill a process and all its children (needed for Dask worker processes).

    Process group killing explained:
    --------------------------------
    When we spawn marimo with `start_new_session=True`, it creates a new session
    and process group. The marimo process becomes the leader, and any children it
    spawns (Dask workers, etc.) inherit the same process group ID (pgid).

    By calling `os.killpg(pgid, signal)`, we send the signal to ALL processes in
    that group simultaneously - this is the standard Unix pattern for cleaning up
    process trees. Without this, killing just the parent would orphan the children.

    The two-phase approach (SIGTERM then SIGKILL) follows best practices:
    - SIGTERM allows graceful shutdown (Dask can close connections, flush data)
    - SIGKILL force-kills any processes that ignore SIGTERM or hang during cleanup

    See: https://alexandra-zaharia.github.io/posts/kill-subprocess-and-its-children-on-timeout-python/
    """
    try:
        pgid = os.getpgid(proc.pid)
    except (ProcessLookupError, OSError):
        return  # Process already dead

    # Phase 1: Graceful shutdown via SIGTERM
    try:
        os.killpg(pgid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        pass

    # Give Dask workers time to shut down gracefully
    time.sleep(1.0)

    # Phase 2: Force kill any remaining processes
    try:
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        pass

    # Wait for main process and clean up pipes (recommended by Python docs)
    try:
        proc.communicate(timeout=3)
    except (subprocess.TimeoutExpired, Exception):
        pass


def on_pre_build(config):
    """Export marimo notebooks to HTML before mkdocs build."""
    for notebook in MARIMO_NOTEBOOKS:
        source = Path(notebook["source"])
        output = Path(notebook["output"])

        if not source.exists():
            print(f"Warning: Notebook {source} not found, skipping")
            continue

        # Skip if output exists and is newer than source
        if output.exists() and output.stat().st_mtime > source.stat().st_mtime:
            print(f"Skipping {source} (output is up to date)")
            continue

        print(f"Exporting {source} -> {output}")
        _export_notebook(source, output)
