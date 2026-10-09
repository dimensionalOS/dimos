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

import asyncio
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import docs, models


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/docs/links",
        response_model=models.DocLinks,
        **route_doc(
            "docs",
            "Links into dimos's published docs: configuring a robot, adding one, blueprints, modules, install",
            "Each link is the docs-site URL of the checkout's page by that name (docs/**/configuration.md, ...); null when there's no such page. No side effects.",
            agent=True,
            answer="`{ site, repo, configure_robot, custom_robot, blueprints, modules, installation, quickstart, cli }`",
        ),
    )
    async def doc_links() -> dict[str, Any]:
        return await asyncio.to_thread(docs.links, s.dimos_dir)
