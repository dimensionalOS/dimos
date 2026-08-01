ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG BUILDER_REQUIREMENTS_SHA256
ARG BUILDER_BASE_DIGEST
ARG BUILDER_VERIFIER_SHA256
ARG BUILDER_PLATFORM

LABEL io.dimos.builder="true" \
      io.dimos.builder-base-digest="${BUILDER_BASE_DIGEST}" \
      io.dimos.builder-requirements-sha256="${BUILDER_REQUIREMENTS_SHA256}" \
      io.dimos.builder-verifier-sha256="${BUILDER_VERIFIER_SHA256}" \
      io.dimos.builder-platform="${BUILDER_PLATFORM}" \
      io.dimos.builder-python="measured-at-build"

RUN apt-get update \
    && apt-get install --no-install-recommends --yes binutils build-essential ca-certificates patchelf \
    && rm -rf /var/lib/apt/lists/*

COPY builder-requirements.lock /tmp/builder-requirements.lock
COPY builder_verify.py /usr/local/bin/dimos-builder-verify.py

RUN python -m pip install --no-cache-dir uv==0.9.17 \
    && uv pip install --system --require-hashes --requirement /tmp/builder-requirements.lock \
    && rm -rf /tmp/builder-requirements.lock /root/.cache /root/.cache/uv

RUN groupadd --gid 1000 builder \
    && useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash builder \
    && install --directory --owner=1000 --group=1000 --mode=0755 /build /out

USER builder
ENTRYPOINT ["python", "/usr/local/bin/dimos-builder-verify.py"]
