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

"""Shared error presentation for OpenArm Mini commands."""

from typing import NoReturn

import typer

from dimos.teleop.openarm_mini.feetech import OpenArmMiniDependencyError


def exit_for_missing_dependency(error: OpenArmMiniDependencyError) -> NoReturn:
    """Print one actionable dependency error and exit without a traceback."""
    typer.echo(str(error), err=True)
    raise typer.Exit(1)
