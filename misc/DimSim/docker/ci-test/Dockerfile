FROM debian:bookworm-slim

# System deps: Playwright browser deps + multicast tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl unzip ca-certificates iproute2 procps \
    fonts-liberation libnss3 libatk-bridge2.0-0 \
    libdrm2 libxcomposite1 libxdamage1 libxrandr2 libgbm1 \
    libpango-1.0-0 libcairo2 libasound2 libxshmfence1 \
    libx11-xcb1 libxcb1 libxext6 libxfixes3 libxi6 \
    libxrender1 libxtst6 libglib2.0-0 libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

# Install Deno
RUN curl -fsSL https://deno.land/install.sh | sh
ENV DENO_DIR=/root/.deno
ENV PATH="/root/.deno/bin:$PATH"

WORKDIR /app

# DimSim — pre-built frontend + CLI tooling
COPY DimSim/dist/         ./DimSim/dist/
COPY DimSim/dimos-cli/    ./DimSim/dimos-cli/
COPY DimSim/evals/        ./DimSim/evals/

# dimos — TS language interop controller
COPY dimos/examples/language-interop/ts/ ./dimos/examples/language-interop/ts/

# Entrypoint
COPY DimSim/docker/ci-test/run-test.sh ./run-test.sh
RUN chmod +x run-test.sh

# Pre-cache Deno deps and install Playwright's Chromium
RUN cd DimSim && deno cache --unstable-net dimos-cli/cli.ts || true
RUN cd /app/dimos/examples/language-interop/ts && deno cache --unstable-net main_custom_multicast.ts || true
RUN cd DimSim && deno run -A npm:playwright install --with-deps chromium || true

ENTRYPOINT ["./run-test.sh"]
