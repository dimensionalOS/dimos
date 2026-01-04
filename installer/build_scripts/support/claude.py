#!/usr/bin/env python3
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

"""Claude helper utilities with simple concurrency controls."""

from __future__ import annotations

import asyncio
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable


async def run_claude_named_prompts(
    prompts: Iterable[tuple[str, str]],
    *,
    max_concurrent: int = 5,
    log_dir: Path | str = Path("./.claude"),
) -> list[int]:
    """
    Execute Claude prompts concurrently and record logs.

    Example:
        >>> asyncio.run(run_claude_named_prompts([], max_concurrent=2))
        []
    """
    if shutil.which("claude") is None:
        raise FileNotFoundError("claude command is required but not available in PATH")

    prompt_list = list(prompts)
    if not prompt_list:
        return []

    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    semaphore = asyncio.Semaphore(max_concurrent)
    results: list[int] = [1] * len(prompt_list)

    async def runner(idx: int, name: str, prompt: str) -> None:
        async with semaphore:
            print(f"starting: {name}")
            log_file_path = log_path / f"{name}.log"
            log_file = log_file_path.open("w", encoding="utf-8")
            try:
                proc = await asyncio.create_subprocess_exec(
                    "claude",
                    "--allowedTools",
                    "Edit,Write,Read,Bash",
                    "-p",
                    prompt,
                    stdout=log_file,
                    stderr=asyncio.subprocess.STDOUT,
                )
                await proc.wait()
                results[idx] = proc.returncode
            finally:
                log_file.close()
            print(f"finished: {name}")

    tasks = [
        asyncio.create_task(runner(idx, name, prompt))
        for idx, (name, prompt) in enumerate(prompt_list)
    ]
    await asyncio.gather(*tasks)
    return results
