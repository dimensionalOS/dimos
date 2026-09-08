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

"""Prepare native executables without starting a robot."""

import typer

from dimos.core.native_package import ensure_native_package

native_app = typer.Typer(help="Prepare native packages through Nix and Cachix.")


@native_app.command()
def prepare(package_id: str) -> None:
    """Download or build PACKAGE_ID using the same inputs as automatic startup."""
    try:
        executable = ensure_native_package(package_id)
    except (ValueError, RuntimeError) as error:
        raise typer.BadParameter(str(error)) from error
    typer.echo(str(executable))
