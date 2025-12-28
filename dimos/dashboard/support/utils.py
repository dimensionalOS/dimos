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


def ensure_logger(logger: logging.Logger | None, log_name: str = "proxy") -> logging.Logger:
    if not logger:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
        )
        return logging.getLogger(log_name)
    else:
        return logger


def make_constant_across_workers(json_data):
    import json

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


class FileBasedBoolean:
    def __init__(self, path):
        import tempfile

        self._path = path or tempfile.NamedTemporaryFile(delete=False).name

    def set(self, value):
        if value:
            with open(self._path, "w+") as the_file:
                the_file.write("1")
        else:
            try:
                os.unlink(self._path)
            except Exception:
                pass

    def get(self):
        from pathlib import Path

        return Path(self._path).exists()

    def clean(self):
        try:
            os.unlink(self._path)
        except Exception:
            pass
