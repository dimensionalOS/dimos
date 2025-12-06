from typing import Optional
import os
import logging

from yarl import URL

def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}

def normalize_path_prefix(prefix: str) -> str:
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    return prefix.rstrip("/") or "/"

def path_matches(prefix: str, path: str) -> bool:
    return path == prefix or path.startswith(prefix + "/")

def build_target_url(
    request: web.Request,
    target_base: str,
    strip_prefix: Optional[str] = None,
    add_prefix: Optional[str] = None,
) -> URL:
    target = URL(target_base)
    path = request.rel_url.path

    if strip_prefix and path_matches(strip_prefix, path):
        path = path[len(strip_prefix) :] or "/"
        if not path.startswith("/"):
            path = "/" + path

    if add_prefix:
        add_prefix = add_prefix.rstrip("/")
        path = f"{add_prefix}{path}"

    full_path = target.path.rstrip("/") + path
    return target.with_path(full_path or "/").with_query(request.rel_url.query)


def ensure_logger(logger: Optional[logging.Logger], log_name: str = "proxy") -> logging.Logger:
    if not logger:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
        )
        return logging.getLogger("proxy")
    else:
        return logger
