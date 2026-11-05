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

import multiprocessing

import pytest


@pytest.fixture
def run_in_subprocess():
    """Bound deadlock checks, including cleanup that can itself block."""

    def run(target, *args):
        process = multiprocessing.get_context("spawn").Process(target=target, args=args)
        try:
            process.start()
            process.join(timeout=10.0)
            assert process.exitcode == 0, f"{target.__name__}: exit code {process.exitcode}"
        finally:
            if process.is_alive():
                process.kill()
                process.join(timeout=2.0)
            process.close()

    return run
