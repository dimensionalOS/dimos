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

"""Resolve registered generated message types without importing legacy packages."""

from dimos_cdrgen.registry import message_types


def resolve_message_type(name: str) -> type | None:
    """Resolve a canonical ROS name or a package.Type shorthand."""
    registered = message_types()
    if name in registered:
        return registered[name]
    if "/" not in name and name.count(".") == 1:
        package, message = name.split(".")
        return registered.get(f"{package}/msg/{message}")
    return None


def resolve_declared_message_type(name: str) -> type | None:
    """Resolve a declared type from a channel/URI.

    Generated types win; legacy ``dimos.msgs`` names still resolve so existing
    channel strings keep working while consumers migrate opt-in.
    """
    resolved = resolve_message_type(name)
    if resolved is not None:
        return resolved
    return resolve_legacy_message_type(name)


def resolve_legacy_message_type(name: str) -> type | None:
    """Resolve a legacy ``dimos.msgs`` type name.

    Channel strings that embed a type suffix (``topic#pkg.Type``) or a key
    expression segment are only produced by legacy publishers, whose payload
    is LCM; the legacy class must win there. The legacy wrapper package is
    retired at the final migration layer, so a missing package simply means
    no legacy spelling can be resolved (return ``None`` rather than raising).
    """
    try:
        from dimos.msgs.helpers import resolve_msg_type
    except ImportError:
        return None

    return resolve_msg_type(name)


def resolve_channel_message_type(name: str) -> type | None:
    """Resolve a type name embedded in a legacy channel/URI (``pkg.Type``).

    The dotted form is what legacy publishers append to channels and key
    expressions, so the legacy class wins there; canonical generated names
    (``pkg/msg/Type``) resolve through the registry.
    """
    if "/" not in name:
        legacy = resolve_legacy_message_type(name)
        if legacy is not None:
            return legacy
    return resolve_declared_message_type(name)
