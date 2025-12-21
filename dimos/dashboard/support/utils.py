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

import logging
import os

from aiohttp import web
from yarl import URL


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def path_matches(prefix: str, path: str) -> bool:
    return path == prefix or path.startswith(prefix + "/")


def build_target_url(
    request: web.Request,
    target_base: str,
    strip_prefix: str | None = None,
    add_prefix: str | None = None,
) -> URL:
    target = URL(target_base)
    path = request.rel_url.path

    if strip_prefix and path_matches(strip_prefix, path):
        path = path[len(strip_prefix) :] or "/"
        if not path.startswith("/"):
            path = "/" + path

    if add_prefix:
        add_prefix = add_prefix.rstrip("/")
        path = f"{add_prefix}{path}"

    full_path = target.path.rstrip("/") + path
    return target.with_path(full_path or "/").with_query(request.rel_url.query)


def ensure_logger(logger: logging.Logger | None, log_name: str = "proxy") -> logging.Logger:
    if not logger:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
        )
        return logging.getLogger("proxy")
    else:
        return logger


def make_constants(json_data):
    import json
    import os

    import psutil

    for each in psutil.Process(os.getpid()).parents():
        try:
            with open(f"/tmp/{each.pid}.json") as infile:
                return json.load(infile)
        except Exception:
            pass
    # if none of the parents have a json file, make one
    with open(f"/tmp/{os.getpid()}.json", "w") as outfile:
        json.dump(json_data, outfile)
    return json_data
