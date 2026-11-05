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

from dimos.utils.threadpool import run_in_thread


def test_run_in_thread_delivers_the_result() -> None:
    assert run_in_thread(lambda: 41 + 1, "adder").result(timeout=5.0) == 42


def test_run_in_thread_delivers_the_exception() -> None:
    def fail() -> None:
        raise RuntimeError("boom")

    future = run_in_thread(fail, "failer")
    assert isinstance(future.exception(timeout=5.0), RuntimeError)
