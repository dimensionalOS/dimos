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

import concurrent.futures
from contextlib import contextmanager
from pathlib import Path
import subprocess
import time

import psutil


@contextmanager
def _managed_process(*args, **kwargs):
    """Context manager that ensures process tree cleanup on exit.

    Uses psutil to recursively kill all child processes. This is cross-platform
    (works on Windows, Linux, macOS) unlike os.killpg which is Unix-only.

    Based on: https://gist.github.com/jizhilong/6687481#gistcomment-3057122
    """
    proc = subprocess.Popen(*args, **kwargs)
    try:
        yield proc
    finally:
        try:
            parent = psutil.Process(proc.pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()
        except psutil.NoSuchProcess:
            pass  # Already dead
        proc.wait()


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
]


def _export_notebook(source: Path, output: Path, timeout: int = 180) -> bool:
    """Export a marimo notebook, killing the process once the file is ready.

    The notebooks use Dask which hangs on shutdown, but the HTML is generated
    within the first few seconds. This function polls for the output file and
    kills the process early once it's ready, rather than waiting for the full timeout.

    Returns True if the export succeeded, False otherwise.
    """
    name = source.stem  # Short name for log messages
    print(f"  [{name}] Exporting", end="", flush=True)

    # Delete old output so we can detect when new file is written
    # (mtime comparison is unreliable due to filesystem timestamp granularity)
    if output.exists():
        output.unlink()

    start = time.time()
    poll_interval = 0.5  # Check every 500ms
    min_file_size = 1000  # HTML should be at least 1KB
    last_size = 0
    stable_count = 0
    stable_threshold = 2  # Require 2 consecutive identical sizes (write_text isn't atomic)
    last_dot_time = start
    dot_interval = 5  # Print a dot every 5 seconds to show activity

    with _managed_process(
        ["marimo", "export", "html", str(source), "-o", str(output), "--force"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ) as proc:
        while time.time() - start < timeout:
            elapsed = time.time() - start

            # Check if process finished naturally
            if proc.poll() is not None:
                stdout = proc.stdout.read().decode() if proc.stdout else ""
                stderr = proc.stderr.read().decode() if proc.stderr else ""
                if proc.returncode == 0:
                    print(f" done ({elapsed:.1f}s)")
                    return True
                else:
                    print(" failed!")
                    print(f"  [{name}] Export failed (exit code {proc.returncode})")
                    if stderr:
                        print(f"  [{name}] stderr: {stderr[:500]}")
                    if stdout:
                        print(f"  [{name}] stdout: {stdout[:500]}")
                    return False

            # Print dots to show activity (works well with captured output)
            if elapsed - (last_dot_time - start) >= dot_interval:
                print(".", end="", flush=True)
                last_dot_time = time.time()

            # Check if output file is ready (exists, stable size, and has content)
            if output.exists():
                current_size = output.stat().st_size
                if current_size > min_file_size and current_size == last_size:
                    stable_count += 1
                    if stable_count >= stable_threshold:
                        # File write is complete - context manager handles cleanup
                        print(f" done ({elapsed:.1f}s, {current_size // 1024}KB)")
                        return True
                else:
                    stable_count = 0
                last_size = current_size

            time.sleep(poll_interval)

        # Timeout reached
        elapsed = time.time() - start
        if output.exists() and output.stat().st_size > min_file_size:
            print(
                f" done ({elapsed:.1f}s, {output.stat().st_size // 1024}KB) [timeout but file generated]"
            )
            return True
        else:
            print(f" timeout ({int(elapsed)}s) - file not generated")
            return False


def on_pre_build(config):
    """Export marimo notebooks to HTML before mkdocs build.

    Exports run in parallel using ThreadPoolExecutor. This is safe because
    each notebook writes to a separate output file and psutil operations are thread-safe.
    """
    to_export: list[tuple[Path, Path]] = []

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

        to_export.append((source, output))

    if not to_export:
        return

    print(f"Exporting {len(to_export)} notebook(s) in parallel...")

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(to_export)) as executor:
        futures = {executor.submit(_export_notebook, src, out): src for src, out in to_export}

        for future in concurrent.futures.as_completed(futures):
            source = futures[future]
            try:
                success = future.result()
                if not success:
                    print(f"Warning: Export failed for {source}")
            except Exception as e:
                print(f"Error exporting {source}: {e}")
