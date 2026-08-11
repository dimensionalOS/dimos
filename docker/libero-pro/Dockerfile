FROM python:3.10-slim-bookworm

ARG LIBERO_PRO_REVISION=eafdb809426b13153aa1e4c42d6601844217dfec

ENV DEBIAN_FRONTEND=noninteractive \
    MUJOCO_GL=egl \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    git libegl1 libgl1 libglib2.0-0 libglfw3 libosmesa6 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY docker/libero-pro/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt
RUN git clone --depth 1 https://github.com/Zxy-MLlab/LIBERO-PRO.git /opt/libero-pro && \
    git -C /opt/libero-pro checkout "$LIBERO_PRO_REVISION" && \
    test "$(git -C /opt/libero-pro rev-parse HEAD)" = "$LIBERO_PRO_REVISION" && \
    pip install --no-cache-dir -e /opt/libero-pro
ENV LIBERO_CONFIG_PATH=/opt/libero-config
COPY docker/libero-pro/config.yaml /opt/libero-config/config.yaml

COPY docker/libero-pro/server.py /app/server.py
COPY dimos/benchmark/libero_pro/proto /app/dimos/benchmark/libero_pro/proto

EXPOSE 50051 50052
CMD ["python", "/app/server.py"]
