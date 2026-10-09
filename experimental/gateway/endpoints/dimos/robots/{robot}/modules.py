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

from typing import Annotated, Any

from fastapi import FastAPI, Path as PathParam

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    discovery = state.discovery
    assert discovery is not None

    @app.get(
        "/dimos/robots/{robot}/modules",
        response_model=models.RobotModules,
        **route_doc(
            "discovery",
            "The modules a robot's blueprints use, most specific to that robot first (TF-IDF style)",
            "score = (fraction of the robot's importable blueprints that use the module) x ln(robots / robots using the module): its own connection module comes first, a module every robot uses scores 0, and a widely shared one (RerunBridgeModule) well below the robot's own. A robot is a annotations.json id (GET /dimos/robots); a blueprint belongs to the robot that lists it or whose dirs hold its file. From the discovery cache; 404 for a robot with no blueprint.",
            errors=(404,),
            agent=True,
            answer="`{ robot, blueprints, blueprints_importable, robots_total, formula, modules: [{ name, class, score, in_robot_blueprints, robot_blueprints, robots_using, blueprint_count }] }`",
        ),
    )
    async def robot_modules(
        robot: Annotated[
            str, PathParam(description="a robot id from GET /dimos/robots", examples=["go2"])
        ],
    ) -> dict[str, Any]:
        answer = discovery.robot_modules(robot)
        if answer is None:
            known = ", ".join(sorted(discovery.robots())) or "none scanned yet"
            raise ApiError(404, f"no blueprints for robot {robot} (robots: {known})")
        return answer
