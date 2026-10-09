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

"""One frozen, dependency-checked native rosbags typestore per process.

Initialize before passing message classes to forkserver workers. Provider discovery
reads schemas only; it must never import the generated value modules recursively.
"""

from __future__ import annotations

from hashlib import sha256
from importlib.util import find_spec
import json
from pathlib import Path
import struct
from threading import RLock
from typing import Any, TypeVar, cast, overload

from rosbags.serde.errors import SerdeError
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from rosbags.typesys.store import Typestore

from .providers import providers

MessageT = TypeVar("MessageT")

ABI = "dimos-native-rosbags-0.11.0-v2"
_lock = RLock()
_store: Typestore | None = None
_packages: dict[str, dict[str, Any]] = {}
_unsupported: set[str] = set()
_schemas: dict[str, str] = {}


def _read_packages(
    roots: tuple[Path, ...], discover: bool
) -> dict[str, tuple[dict[str, Any], dict[str, str]]]:
    pending = list(roots)
    if discover:
        pending.extend(Path(provider.package_root()) for provider in providers())
    result: dict[str, tuple[dict[str, Any], dict[str, str]]] = {}
    while pending:
        root = pending.pop()
        manifest = json.loads((root / "message-package.json").read_text())
        module = manifest["module"]
        if manifest.get("abi") != ABI:
            raise ValueError(
                f"Incompatible message package ABI: {module}; rebuild the source package"
            )
        if module in result:
            if result[module][0] != manifest:
                raise ValueError(f"Conflicting message package identity/version: {module}")
            continue
        schemas = json.loads((root / "schemas.json").read_text())
        if {name: sha256(text.encode()).hexdigest() for name, text in schemas.items()} != manifest[
            "schemas"
        ]:
            raise ValueError(f"Message schema digest mismatch: {module}")
        result[module] = (manifest, schemas)
        for dependency in manifest["dependencies"]:
            found = find_spec(dependency + "_schemas")
            if found is None or found.origin is None:
                raise ValueError(f"Missing installed message dependency: {dependency}")
            pending.append(Path(found.origin).parent / "package")
    owners: dict[str, str] = {}
    merged: dict[str, str] = {}
    for module, (manifest, schemas) in result.items():
        for dependency, version in manifest["dependencies"].items():
            if result[dependency][0]["version"] != version:
                raise ValueError(f"Dependency version mismatch: {dependency}=={version}")
        for name in manifest["owned"]:
            if name in owners:
                raise ValueError(f"Multiple owners for message {name}")
            owners[name] = module
        for name, schema in schemas.items():
            if merged.setdefault(name, schema) != schema:
                raise ValueError(f"Dependency schema mismatch: {name}")
    if merged.keys() - owners.keys():
        raise ValueError(f"Missing message owners: {sorted(merged.keys() - owners.keys())}")
    return result


def initialize(roots: tuple[Path, ...] = (), *, discover: bool = True) -> Typestore:
    """Register the complete installed closure once; reject later mutations."""
    global _store
    with _lock:
        packages = _read_packages(roots, discover)
        if _store is not None:
            for module, (manifest, _) in packages.items():
                if _packages.get(module) != manifest:
                    raise RuntimeError(
                        f"Message registry is frozen; restart after installing/changing {module}"
                    )
            return _store
        definitions = {}
        for _manifest, schemas in packages.values():
            for name, text in schemas.items():
                definitions.update(get_types_from_msg(text, name))
        store = get_typestore(Stores.EMPTY)
        store.register(definitions)
        _packages.update({module: manifest for module, (manifest, _) in packages.items()})
        for manifest, schemas in packages.values():
            _schemas.update(schemas)
            _unsupported.update(manifest.get("unsupported", {}).get("python", {}))
        _store = store
        return store


def message_types() -> dict[str, type[Any]]:
    store = _store if _store is not None else initialize()
    return dict(store.types)


def encode(value: Any, *, little_endian: bool = True) -> bytes:
    """Serialize native values; do not silently discard unsupported bounds."""
    store = _store if _store is not None else initialize()
    name = value.__msgtype__
    if name in _unsupported:
        raise NotImplementedError(f"Native Python bounds validation is unsupported for {name}")
    if type(value) is not store.types[name]:
        raise TypeError(f"Message class is not owned by the frozen registry: {name}")
    return bytes(store.serialize_cdr(value, name, little_endian=little_endian))


@overload
def decode(data: bytes, name: type[MessageT]) -> MessageT: ...


@overload
def decode(data: bytes, name: str) -> Any: ...


def decode(data: bytes, name: str | type[MessageT]) -> Any:
    typename = name if isinstance(name, str) else cast("Any", name).__msgtype__
    if not isinstance(typename, str):
        raise TypeError("Message type must supply a string __msgtype__")
    store = _store if _store is not None else initialize()
    if typename in _unsupported:
        raise NotImplementedError(f"Native Python bounds validation is unsupported for {typename}")
    try:
        return store.deserialize_cdr(data, typename)
    except (SerdeError, struct.error, AssertionError, IndexError) as error:
        raise ValueError(f"Invalid CDR message for {typename}: {error}") from error


def schema(name: str) -> str:
    """Return the original complete ros2msg closure, including imported types."""
    if _store is None:
        initialize()
    return _schemas[name]
