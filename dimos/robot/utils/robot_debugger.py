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

import os

from dimos.core.resource import Resource
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RobotDebugger(Resource):
    def __init__(self, robot) -> None:  # type: ignore[no-untyped-def]
        self._robot = robot
        self._threaded_server = None

    def start(self) -> None:
        if not os.getenv("ROBOT_DEBUGGER"):
            return

        try:
            import rpyc  # type: ignore[import-not-found]
            from rpyc.utils.server import ThreadedServer  # type: ignore[import-not-found]
        except ImportError:
            return

        logger.info(
            "Starting the robot debugger. You can open a python shell with `./bin/robot-debugger`"
        )

        robot = self._robot

        class RobotService(rpyc.Service):  # type: ignore[misc]
            def exposed_robot(self):  # type: ignore[no-untyped-def]
                return robot

        self._threaded_server = ThreadedServer(
            RobotService,
            port=18861,
            protocol_config={
                "allow_all_attrs": True,
            },
        )
        self._threaded_server.start()  # type: ignore[attr-defined]

    def stop(self) -> None:
        if self._threaded_server:
            self._threaded_server.close()
