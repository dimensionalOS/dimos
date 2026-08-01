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

# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Advisory package configuration and normalized audit evidence.

The selected proxy and index are recorded explicitly for reproducibility. This
configuration does not enforce egress policy or prevent data exfiltration.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from urllib.parse import urlsplit


class PackageIndexConfigurationError(ValueError):
    """Raised for malformed advisory package configuration or audit data."""


def _host(url: str) -> str:
    parsed = urlsplit(url)
    if parsed.scheme != "https" or not parsed.hostname:
        raise ProxyConfigurationError("proxy and package indexes must be HTTPS URLs")
    if parsed.username or parsed.password or parsed.fragment:
        raise ProxyConfigurationError("credentials and fragments are not allowed in URLs")
    return parsed.hostname.lower().rstrip(".")


@dataclass(frozen=True)
class PackageIndexConfig:
    """User-selected package settings and advisory host allowlist."""

    proxy_url: str
    index_urls: tuple[str, ...]
    allowed_hosts: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.index_urls or not self.allowed_hosts:
            raise ProxyConfigurationError("at least one index and allowed host are required")
        proxy_host = _host(self.proxy_url)
        hosts = frozenset(host.lower().rstrip(".") for host in self.allowed_hosts)
        if proxy_host not in hosts:
            raise ProxyConfigurationError("proxy host is not allowlisted")
        for index in self.index_urls:
            if _host(index) not in hosts:
                raise ProxyConfigurationError("package index host is not allowlisted")

    @property
    def normalized(self) -> dict[str, str | tuple[str, ...]]:
        return {
            "proxy_url": self.proxy_url.rstrip("/"),
            "index_urls": tuple(url.rstrip("/") for url in self.index_urls),
            "allowed_hosts": tuple(
                sorted({host.lower().rstrip(".") for host in self.allowed_hosts})
            ),
        }


@dataclass(frozen=True)
class PackageIndexAuditEvidence:
    """Normalized package audit data; it is not a network security control."""

    config: PackageIndexConfig
    requested_urls: tuple[str, ...]

    def normalized(self) -> dict[str, object]:
        allowed = {host.lower().rstrip(".") for host in self.config.allowed_hosts}
        urls = tuple(url.rstrip("/") for url in self.requested_urls)
        for url in urls:
            if _host(url) not in allowed:
                raise ProxyConfigurationError("observed URL host is not allowlisted")
        return {
            "config": self.config.normalized,
            "requested_urls": urls,
        }


@dataclass(frozen=True)
class PostRunCommandAudit:
    """Observable command findings, never proof that online use was absent."""

    findings: tuple[str, ...]
    limitation: str = "This audit cannot prove absence of online use."


_AUDIT_PATTERNS: tuple[tuple[str, str], ...] = (
    ("network-shell", r"\b(?:curl|wget|nc|ncat|ssh|git\s+clone)\b"),
    ("package-network", r"\b(?:pip|python\s+-m\s+pip|uv\s+(?:pip|run))\b"),
    ("proxy-override", r"(?i)(?:--proxy|(?:https?|all)_proxy\s*=|PIP_PROXY\s*=)"),
    ("index-override", r"(?i)(?:--(?:index-url|extra-index-url)|(?:PIP|UV)_INDEX_URL\s*=)"),
)


def audit_post_run_output(output: str) -> PostRunCommandAudit:
    """Flag observable network commands and proxy/index overrides.

    A clean result only means patterns were not observed in supplied output;
    it cannot prove that the workload made no online requests.
    """

    findings = tuple(name for name, pattern in _AUDIT_PATTERNS if re.search(pattern, output))
    return PostRunCommandAudit(findings)


# Compatibility aliases for the initial advisory-only names.
PackageIndexProxyConfig = PackageIndexConfig
ProxyEvidence = PackageIndexAuditEvidence
ProxyConfig = PackageIndexConfig
ProxyConfigurationError = PackageIndexConfigurationError
