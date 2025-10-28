#!/usr/bin/env python3

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

from dataclasses import dataclass

from dimos.protocol.service.spec import Service


@dataclass
class DatabaseConfig:
    host: str = "localhost"
    port: int = 5432
    database_name: str = "test_db"
    timeout: float = 30.0
    max_connections: int = 10
    ssl_enabled: bool = False


class DatabaseService(Service[DatabaseConfig]):
    default_config = DatabaseConfig

    def start(self) -> None: ...
    def stop(self) -> None: ...


def test_default_configuration() -> None:
    """Test that default configuration is applied correctly."""
    service = DatabaseService()

    # Check that all default values are set
    assert service.config.host == "localhost"
    assert service.config.port == 5432
    assert service.config.database_name == "test_db"
    assert service.config.timeout == 30.0
    assert service.config.max_connections == 10
    assert service.config.ssl_enabled is False


def test_partial_configuration_override() -> None:
    """Test that partial configuration correctly overrides defaults."""
    service = DatabaseService(host="production-db", port=3306, ssl_enabled=True)

    # Check overridden values
    assert service.config.host == "production-db"
    assert service.config.port == 3306
    assert service.config.ssl_enabled is True

    # Check that defaults are preserved for non-overridden values
    assert service.config.database_name == "test_db"
    assert service.config.timeout == 30.0
    assert service.config.max_connections == 10


def test_complete_configuration_override() -> None:
    """Test that all configuration values can be overridden."""
    service = DatabaseService(
        host="custom-host",
        port=9999,
        database_name="custom_db",
        timeout=60.0,
        max_connections=50,
        ssl_enabled=True,
    )

    # Check that all values match the custom config
    assert service.config.host == "custom-host"
    assert service.config.port == 9999
    assert service.config.database_name == "custom_db"
    assert service.config.timeout == 60.0
    assert service.config.max_connections == 50
    assert service.config.ssl_enabled is True


def test_service_subclassing() -> None:
    @dataclass
    class ExtraConfig(DatabaseConfig):
        extra_param: str = "default_value"

    class ExtraDatabaseService(DatabaseService):
        default_config = ExtraConfig

        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)

    bla = ExtraDatabaseService(host="custom-host2", extra_param="extra_value")

    assert bla.config.host == "custom-host2"
    assert bla.config.extra_param == "extra_value"
    assert bla.config.port == 5432  # Default value from DatabaseConfig
