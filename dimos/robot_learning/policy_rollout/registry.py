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

"""Lazy registries for robot policy rollout backends and contracts."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import importlib
import os
from typing import Generic, TypeVar, cast

from dimos.robot_learning.policy_rollout.backends.backend import PolicyBackend
from dimos.robot_learning.policy_rollout.contract import RobotPolicyContract

FactoryT = TypeVar("FactoryT")
BackendFactory = Callable[..., PolicyBackend]
ContractFactory = Callable[..., RobotPolicyContract]


class _LazyFactoryRegistry(Generic[FactoryT]):
    def __init__(
        self,
        *,
        package: str,
        manifest_name: str,
        manifest_attr: str,
        item_label: str,
    ) -> None:
        self._package = package
        self._manifest_name = manifest_name
        self._manifest_attr = manifest_attr
        self._item_label = item_label
        self._factory_paths: dict[str, str] = {}
        self._factories: dict[str, FactoryT] = {}
        self.discover()

    def discover(self) -> None:
        """Discover registry manifests without importing heavy implementations."""

        package = importlib.import_module(self._package)
        for root in package.__path__:
            for entry in sorted(os.listdir(root)):
                if entry.startswith(("_", ".")):
                    continue
                entry_path = os.path.join(root, entry)
                if not os.path.isdir(entry_path):
                    continue
                module_name = f"{self._package}.{entry}.{self._manifest_name}"
                try:
                    module = importlib.import_module(module_name)
                except ModuleNotFoundError as exc:
                    if exc.name == module_name:
                        continue
                    raise
                factories = getattr(module, self._manifest_attr, None)
                if not isinstance(factories, Mapping):
                    raise TypeError(f"{module_name} must define {self._manifest_attr}")
                for name, factory_path in factories.items():
                    if not isinstance(name, str) or not isinstance(factory_path, str):
                        raise TypeError(
                            f"{module_name}.{self._manifest_attr} must map strings to strings"
                        )
                    self.register_path(name, factory_path)

    def register_path(self, name: str, factory_path: str) -> None:
        if ":" not in factory_path:
            raise ValueError(f"Invalid {self._item_label} factory path: {factory_path!r}")
        key = name.lower()
        existing = self._factory_paths.get(key)
        if existing is not None and existing != factory_path:
            raise ValueError(
                f"Duplicate {self._item_label} type {key!r}: {existing!r} vs {factory_path!r}"
            )
        self._factory_paths[key] = factory_path

    def create(self, name: str, **params: object) -> FactoryT:
        key = name.lower()
        factory = self._resolve_factory(key)
        callable_factory = cast("Callable[..., FactoryT]", factory)
        return callable_factory(**params)

    def available(self) -> list[str]:
        return sorted(self._factory_paths.keys())

    def _resolve_factory(self, key: str) -> FactoryT:
        if key in self._factories:
            return self._factories[key]
        if key not in self._factory_paths:
            raise ValueError(
                f"Unknown {self._item_label} type: {key!r}. Available: {self.available()}"
            )
        factory_path = self._factory_paths[key]
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            raise ValueError(
                f"{self._item_label.title()} {key!r} is registered to missing module "
                f"{module_name!r}"
            ) from exc
        factory = getattr(module, attr)
        if not callable(factory):
            raise TypeError(f"{self._item_label.title()} factory {factory_path!r} is not callable")
        typed_factory = cast("FactoryT", factory)
        self._factories[key] = typed_factory
        return typed_factory


class PolicyBackendRegistry(_LazyFactoryRegistry[BackendFactory]):
    def __init__(self) -> None:
        super().__init__(
            package="dimos.robot_learning.policy_rollout.backends",
            manifest_name="__registry__",
            manifest_attr="POLICY_BACKENDS",
            item_label="policy backend",
        )


class RobotPolicyContractRegistry(_LazyFactoryRegistry[ContractFactory]):
    def __init__(self) -> None:
        super().__init__(
            package="dimos.robot_learning.policy_rollout.contracts",
            manifest_name="__registry__",
            manifest_attr="ROBOT_POLICY_CONTRACTS",
            item_label="robot policy contract",
        )


policy_backend_registry = PolicyBackendRegistry()
robot_policy_contract_registry = RobotPolicyContractRegistry()
