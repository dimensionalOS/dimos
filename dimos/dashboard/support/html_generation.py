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

from functools import lru_cache
import os
import re
from typing import Optional

from jinja2 import Environment, FileSystemLoader, select_autoescape

DEFAULT_SESSION_NAME = "dimos-dashboard"

path_to_baseline_css = os.path.join(os.path.dirname(__file__), "css_baseline.css")
with open(path_to_baseline_css) as f:
    css_baseline_contents = f.read()

session_name_regex = re.compile(r"^[A-Za-z0-9_-]+$")


def ensure_session_name_valid(value: str) -> str:
    """
    Note: this function is enforcing two restrictions:
        - the value must be valid if embedded html attribute (no double quotes)
        - the value must be valid as a zellij session name
    """
    if not isinstance(value, str):
        raise TypeError(f"Expected str, got {type(value).__name__}")
    if not session_name_regex.match(value):
        raise ValueError(
            "session name may only contain letters, numbers, underscores, or dashes. Got " + value
        )
    if len(value) < 2:
        raise ValueError("session name must be at least 2 characters long. Got: " + value)

    return value


template_dir = os.path.join(os.path.dirname(__file__), "templates")
jinja_env = Environment(
    loader=FileSystemLoader(template_dir),
    autoescape=select_autoescape(["html", "xml"]),
    trim_blocks=True,
    lstrip_blocks=True,
)


@lru_cache(maxsize=2)
def html_code_gen(
    rrd_url: str,
    zellij_enabled: bool = True,
    zellij_token: str | None = None,
    session_name: str = DEFAULT_SESSION_NAME,
) -> str:
    # TODO: download "https://esm.sh/@rerun-io/web-viewer@0.27.2" so that rerun works offline

    session_name = ensure_session_name_valid(session_name)
    template = jinja_env.get_template("dashboard.html")
    return template.render(
        css_baseline=css_baseline_contents,
        rrd_url=rrd_url,
        zellij_enabled=zellij_enabled,
        zellij_token=zellij_token,
        session_name=session_name,
    )
