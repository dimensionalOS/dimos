#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
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

# Configurable auto-resolves config type from generic params.
#
# Generic chain (TypeVars pass through, concrete params resolve at init):
#
#   Configurable[T]  →  Service[T]  →  DBService[T]  →  ProdDBService
#                                           ↑                  ↑
#                                      T bound=DBConfig   [ProdDBConfig]

from typing import TypeVar

import pytest

from dimos.protocol.service.spec import BaseConfig, Service


class DBConfig(BaseConfig):
    host: str = "localhost"
    port: int = 5432


class ProdDBConfig(DBConfig):
    pool_size: int = 20


T = TypeVar("T", bound=DBConfig)


class DBService(Service[T]):
    """Generic — subclasses concretize with their own config."""

    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        super().stop()


class ProdDBService(DBService[ProdDBConfig]):
    """Concretizes T → ProdDBConfig."""


class LocalDBService(DBService[DBConfig]):
    """Concretizes T → DBConfig."""


def test_instantiate_with_defaults() -> None:
    svc = LocalDBService()
    assert isinstance(svc.config, DBConfig)
    assert svc.config.host == "localhost"
    assert svc.config.port == 5432


def test_instantiate_with_overrides() -> None:
    svc = LocalDBService(host="dev-db", port=3306)
    assert svc.config.host == "dev-db"
    assert svc.config.port == 3306


def test_subclass_gets_extended_config() -> None:
    svc = ProdDBService()
    assert isinstance(svc.config, ProdDBConfig)
    assert svc.config.pool_size == 20
    assert svc.config.host == "localhost"


def test_plain_subclass_inherits_config() -> None:
    class StagingDBService(LocalDBService):
        pass

    svc = StagingDBService()
    assert isinstance(svc.config, DBConfig)


def test_typevar_default_fallback() -> None:
    """Unparameterized subclass falls back to TypeVar default."""
    from typing_extensions import TypeVar as ExtTypeVar

    U = ExtTypeVar("U", bound=DBConfig, default=DBConfig)

    class GenericService(Service[U]):
        def start(self) -> None:
            super().start()

        def stop(self) -> None:
            super().stop()

    class PlainChild(GenericService):
        pass

    svc = PlainChild()
    assert isinstance(svc.config, DBConfig)


def test_no_concrete_config_raises() -> None:
    with pytest.raises(TypeError, match="no concrete config type"):
        DBService()
