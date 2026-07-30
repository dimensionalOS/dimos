---
myst:
  substitutions:
    output: |-
      ```{image} assets/docker-hierarchy.svg
      ```
---

(doc-development-docker-docker-images)=

# Docker Images

DimOS uses parallel Docker image hierarchies for ROS and non-ROS builds, allowing you to choose the environment that fits your use case.

(doc-development-docker-image-hierarchy)=

## Image Hierarchy

```{raw} html
<details>
<summary>Pikchr</summary>
```

```text
color = white
fill = none

# Base images
U1: box "ubuntu:22.04" rad 5px fit wid 170% ht 170%
U2: box "ubuntu:22.04" rad 5px fit wid 170% ht 170% at (U1.x + 2.5in, U1.y)

# Labels
text "Non-ROS Track" at (U1.x, U1.y + 0.5in)
text "ROS Track" at (U2.x, U2.y + 0.5in)

# Non-ROS track
arrow from U1.s down 0.4in
P: box "python" rad 5px fit wid 170% ht 170%
arrow from P.s down 0.4in
D: box "dev" rad 5px fit wid 170% ht 170%

# ROS track
arrow from U2.s down 0.4in
R: box "ros" rad 5px fit wid 170% ht 170%
arrow from R.s down 0.4in
RP: box "ros-python" rad 5px fit wid 170% ht 170%
arrow from RP.s down 0.4in
RD: box "ros-dev" rad 5px fit wid 170% ht 170%

# Cross-reference: same dockerfiles reused
line dashed from P.e right 0.3in then down until even with RP then right to RP.w
line dashed from D.e right 0.3in then down until even with RD then right to RD.w
text "same dockerfiles" at (D.e.x + 1.2in, D.e.y + 0.4in)
```

```{raw} html
</details>
```

{{ output }}

(doc-development-docker-images)=

## Images

All images are published to `ghcr.io/dimensionalos/`.

| Image        | Base         | Purpose                                            |
| ------------ | ------------ | -------------------------------------------------- |
| `python`     | ubuntu:22.04 | Core dimos with Python dependencies, no ROS        |
| `dev`        | python       | Development environment (editors, git, pre-commit) |
| `ros`        | ubuntu:22.04 | ROS2 Humble with navigation packages               |
| `ros-python` | ros          | ROS + dimos Python dependencies                    |
| `ros-dev`    | ros-python   | Full ROS development environment                   |

(doc-development-docker-tags)=

## Tags

Images are tagged based on the git branch:

| Branch           | Tag                                             |
| ---------------- | ----------------------------------------------- |
| `main`           | `latest`                                        |
| `dev`            | `dev`                                           |
| feature branches | sanitized branch name (e.g., `feature_foo_bar`) |

(doc-development-docker-when-to-use-each-image)=

## When to Use Each Image

(non-ros-track-python-dev)=

(doc-development-docker-non-ros-track-python-dev)=

### Non-ROS Track (`python` → `dev`)

```sh
docker run -it ghcr.io/dimensionalos/dev:latest bash
```

(ros-track-ros-ros-python-ros-dev)=

(doc-development-docker-ros-track-ros-ros-python-ros-dev)=

### ROS Track (`ros` → `ros-python` → `ros-dev`)

Use when you need ROS2 integration:

- Robot hardware control via ROS topics
- Navigation stack integration
- ROS message passing between components
- Running ROS tests (`pytest -m ros`)

```sh
docker run -it ghcr.io/dimensionalos/ros-dev:latest bash
```

(doc-development-docker-local-development)=

## Local Development

(doc-development-docker-building-images-locally)=

### Building Images Locally

Use the helper script:

```sh
./bin/dockerbuild python    # Build python image
./bin/dockerbuild dev       # Build dev image
./bin/dockerbuild ros       # Build ros image
```

(doc-development-docker-cicd-pipeline)=

## CI/CD Pipeline

Images are built by [.github/workflows/docker-build.yml](https://github.com/dimensionalOS/dimos/blob/main/.github/workflows/docker-build.yml#L4) on merges to `main`/`dev` (when Docker files change) and weekly for base image security patches.

Tests and type checking run in [.github/workflows/ci.yml](https://github.com/dimensionalOS/dimos/blob/main/.github/workflows/ci.yml) using pre-built images.

(doc-development-docker-build-trigger-paths)=

### Build Trigger Paths

| Image    | Triggers on changes to             |
| -------- | ---------------------------------- |
| `ros`    | `docker/ros/**`, workflow files    |
| `python` | `docker/python/**`, workflow files |
| `dev`    | `docker/dev/**`                    |

(doc-development-docker-dockerfile-structure)=

## Dockerfile Structure

(doc-development-docker-common-patterns)=

### Common Patterns

All Dockerfiles accept a `FROM_IMAGE` build arg for flexibility:

```dockerfile
ARG FROM_IMAGE=ubuntu:22.04
FROM ${FROM_IMAGE}
```

This allows the same Dockerfile (e.g., `python`) to build on different bases.

(doc-development-docker-python-package-installation)=

### Python Package Installation

Images use [uv](https://github.com/astral-sh/uv) for fast dependency installation:

```dockerfile
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
RUN uv pip install '.[misc,cpu,sim,drone,unitree,web,perception,visualization]'
```

(doc-development-docker-dev-image-features)=

### Dev Image Features

The dev image ([docker/dev/Dockerfile](https://github.com/dimensionalOS/dimos/blob/main/docker/dev/Dockerfile)) adds:

- Git, git-lfs, pre-commit
- Editors (nano, vim)
- tmux with custom config
- Node.js (via nvm)
- Custom bash prompt with version info
- Entrypoint script that sources ROS setup
