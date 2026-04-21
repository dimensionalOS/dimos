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

from __future__ import annotations

import threading


def test_skills_discovery(running_app):
    skills = running_app.skills
    assert "echo" in dir(skills)
    assert "ping" in dir(skills)
    rep = repr(skills)
    assert "echo" in rep
    assert "ping" in rep


def test_skill_call(running_app):
    result = running_app.skills.echo(message="hello")
    assert result == "hello"


def test_skill_ping(running_app):
    result = running_app.skills.ping()
    assert result == "pong"


def test_thread_safety(running_app):
    results: list[str] = []
    errors: list[Exception] = []

    def call_skill():
        try:
            r = running_app.skills.ping()
            results.append(r)
        except Exception as e:
            errors.append(e)

    threads = [threading.Thread(target=call_skill) for _ in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)

    assert not errors, f"Errors in threads: {errors}"
    assert len(results) == 5
    assert all(r == "pong" for r in results)
