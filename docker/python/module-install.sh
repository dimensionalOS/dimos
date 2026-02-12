#!/usr/bin/env bash
# DimOS Module Install (generic)
# Converts any Dockerfile into a DimOS module container
#
# Usage in Dockerfile:
#   RUN --mount=from=ghcr.io/dimensionalos/ros-python:dev,source=/app,target=/tmp/d \
#       bash /tmp/d/docker/python/module-install.sh /tmp/d
#   ENTRYPOINT ["/dimos/entrypoint.sh"]

set -euo pipefail

SRC="${1:-/tmp/d}"

# ---- Copy source into image (skip if already at /dimos/source) ----
if [ "${SRC}" != "/dimos/source" ]; then
    mkdir -p /dimos/source
    cp -r "${SRC}/dimos" "${SRC}/pyproject.toml" /dimos/source/
    [ -f "${SRC}/README.md" ] && cp "${SRC}/README.md" /dimos/source/ || true
fi

# ---- Find Python + Pip (conda env > venv > uv > system) ----
PYTHON=""
PIP=""

# 1. Check for Conda environment
if [ -z "$PYTHON" ] && command -v conda >/dev/null 2>&1; then
    DIMOS_CONDA_ENV="${DIMOS_CONDA_ENV:-app}"
    if conda env list 2>/dev/null | awk '{print $1}' | grep -qx "${DIMOS_CONDA_ENV}"; then
        PYTHON="conda run --no-capture-output -n ${DIMOS_CONDA_ENV} python"
        PIP="conda run -n ${DIMOS_CONDA_ENV} pip"
        echo "Using Conda env: ${DIMOS_CONDA_ENV}"
    fi
fi

# 2. Check for venv (including uv's .venv)
if [ -z "$PYTHON" ]; then
    for v in /opt/venv /app/venv /venv /app/.venv /.venv; do
        if [ -x "${v}/bin/python" ] && [ -x "${v}/bin/pip" ]; then
            PYTHON="${v}/bin/python"
            PIP="${v}/bin/pip"
            echo "Using venv: ${v}"
            break
        fi
    done
fi

# 3. Check for uv (uses system python but manages deps)
if [ -z "$PYTHON" ] && command -v uv >/dev/null 2>&1; then
    PYTHON="python"
    PIP="uv pip"
    echo "Using uv"
fi

# 4. Fallback to system Python
if [ -z "$PYTHON" ]; then
    PYTHON="python"
    PIP="pip"
    echo "Using system Python"
fi

# ---- Install DimOS (deps from pyproject.toml[docker]) ----
${PIP} install --no-cache-dir -e "/dimos/source[docker]"

# ---- Create entrypoint ----
cat > /dimos/entrypoint.sh <<EOF
#!/usr/bin/env bash
set -euo pipefail
export PYTHONPATH="/dimos/source:/dimos/third_party:\${PYTHONPATH:-}"
exec ${PYTHON} -m dimos.core.docker_runner run "\$@"
EOF

chmod +x /dimos/entrypoint.sh
echo "DimOS module installed. Use: ENTRYPOINT [\"/dimos/entrypoint.sh\"]"
