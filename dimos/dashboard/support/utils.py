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

from functools import wraps
import logging
import os
import time
from typing import Optional

from aiohttp import web
from yarl import URL


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def normalize_path_prefix(prefix: str) -> str:
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    return prefix.rstrip("/") or "/"


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


def rate_limit(min_interval: float):
    """
    Prevent the function from being called more often than once every `min_interval` seconds.
    """

    def decorator(func):
        last_called = 0.0

        @wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal last_called
            now = time.time()
            if now - last_called < min_interval:
                return  # skip call
            last_called = now
            return func(*args, **kwargs)

        return wrapper

    return decorator


def record_message(path: str, val):
    import base64
    from pathlib import Path
    import pickle

    payload = base64.b64encode(pickle.dumps(val)).decode("ascii")
    line = f'- !!python/object/apply:pickle.loads [!!binary "{payload}"]\n'
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(line)


def load_messages(log_filepath: str) -> list:
    """Read a YAML log created return a list of depickled messages."""
    from pathlib import Path

    import yaml  # PyYAML is a common dependency in the project

    path = Path(log_filepath)
    if not path.exists():
        return []

    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or []

    # Items are already restored by pickle.loads via the YAML tag
    return list(data)


def make_constants(json_data):
    import json
    import os

    import psutil

    for each in psutil.Process(os.getpid()).parents():
        try:
            with open(f"/tmp/{each.pid}.json") as infile:
                return json.load(infile)
        except:
            pass
    # if none of the parents have a json file, make one
    with open(f"/tmp/{os.getpid()}.json", "w") as outfile:
        json.dump(json_data, outfile)
    return json_data
