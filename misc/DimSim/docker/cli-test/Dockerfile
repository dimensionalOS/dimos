FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl unzip ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install Deno
RUN curl -fsSL https://deno.land/install.sh | sh
ENV PATH="/root/.deno/bin:$PATH"

WORKDIR /app

# Install CLI from JSR (the real install path)
RUN deno install -gAf --name dimsim --allow-all --unstable-net jsr:@antim/dimsim

# Copy release tarballs for local setup test
COPY dimsim-core-v0.1.0.tar.gz ./
COPY scene-apt-v0.1.0.tar.gz ./

COPY docker/cli-test/test.sh ./test.sh
RUN chmod +x test.sh

ENTRYPOINT ["./test.sh"]
