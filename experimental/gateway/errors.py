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


class StatusError(Exception):
    """An error the API answers with its `status` and message (app.py's one handler); anything else is a 500."""

    status = 500


class ApiError(StatusError):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
