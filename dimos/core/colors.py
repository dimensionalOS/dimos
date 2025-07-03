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


def green(text: str) -> str:
    """Return the given text in green color."""
    return f"\033[92m{text}\033[0m"


def blue(text: str) -> str:
    """Return the given text in blue color."""
    return f"\033[94m{text}\033[0m"


def red(text: str) -> str:
    """Return the given text in red color."""
    return f"\033[91m{text}\033[0m"


def yellow(text: str) -> str:
    """Return the given text in yellow color."""
    return f"\033[93m{text}\033[0m"


def cyan(text: str) -> str:
    """Return the given text in cyan color."""
    return f"\033[96m{text}\033[0m"


def orange(text: str) -> str:
    """Return the given text in orange color."""
    return f"\033[38;5;208m{text}\033[0m"
