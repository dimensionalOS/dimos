# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import datetime
import json
import random
from typing import Any, Dict, Optional

SPORT_MODE_TOPIC = "rt/api/sport/request"


def generate_id() -> int:
    return int(datetime.datetime.now().timestamp() * 1000 % 2147483648) + random.randint(0, 999)


def create_command_structure(api_id: int, parameter: Any, topic: str = SPORT_MODE_TOPIC) -> Dict:
    return {
        "type": "msg",
        "topic": topic,
        "data": {
            "header": {
                "identity": {
                    "id": generate_id(),
                    "api_id": api_id
                }
            },
            "parameter": json.dumps(parameter)
        }
    }


def gen_command(cmd: int, parameters: Optional[Any] = None, topic: Optional[str] = None) -> str:
    parameter = parameters if parameters is not None else cmd
    command = create_command_structure(
        api_id=cmd,
        parameter=parameter,
        topic=topic or SPORT_MODE_TOPIC
    )
    return json.dumps(command)


def gen_mov_command(x: float, y: float, z: float) -> str:
    parameters = {"x": x, "y": y, "z": z}
    command = create_command_structure(
        api_id=1008,
        parameter=parameters,
        topic=SPORT_MODE_TOPIC
    )
    return json.dumps(command)
