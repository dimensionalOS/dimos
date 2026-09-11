#!/usr/bin/env bash
set -euxo pipefail
uv run pytest --numprocesses=auto dimos
