from pathlib import Path

CONTAINERFILE = Path(__file__).with_name("Containerfile")


def test_containerfile_artifact_contract() -> None:
    text = CONTAINERFILE.read_text()
    assert "ARG BASE_IMAGE" in text and "ARG BASE_IMAGE=" not in text
    assert "COPY requirements.lock /tmp/requirements.lock" in text
    assert "COPY dimos-*.whl /tmp/" in text
    assert "COPY build-manifest.v1.json /tmp/build-manifest.v1.json" in text
    assert "COPY ." not in text
    assert "--no-deps --require-hashes --requirement /tmp/requirements.lock" in text
    assert 'uv pip install --system --no-deps "/tmp/${WHEEL_FILENAME}"' in text
    assert "py3-none-any" not in text
    assert "uv pip check --system" in text
    assert "libturbojpeg0" in text
    assert "dimos.navigation.replanning_a_star.min_cost_astar_ext" in text
    assert "from turbojpeg import TurboJPEG; TurboJPEG()" in text


def test_rootless_runtime_contract() -> None:
    text = CONTAINERFILE.read_text()
    for value in (
        'io.dimos.python="3.12"',
        "USER runner",
        "WORKDIR /work",
        "ARG RUNNER_UID",
        "ARG RUNNER_GID",
        "VIRTUAL_ENV=/agent/venv",
        "PYTHONNOUSERSITE=1",
        "UV_LINK_MODE=copy",
    ):
        assert value in text
    for module in (
        "dimos.benchmark.spatial.models",
        "numpy",
        "scipy",
        "cv2",
        "open3d",
        "pinocchio",
        "rerun",
    ):
        assert module in text
    assert "editable" not in text.lower()
    assert "--no-default-groups" not in text  # export belongs to the build utility
    assert "dimos.perception.spatial_perception" not in text


def test_builder_context_and_runtime_provenance_contract() -> None:
    text = CONTAINERFILE.read_text()
    builder = CONTAINERFILE.with_name("Builder.Containerfile").read_text()
    script = CONTAINERFILE.with_name("builder_verify.py").read_text()
    requirements = CONTAINERFILE.with_name("builder-requirements.lock").read_text()
    for value in (
        "FROM ${BASE_IMAGE}",
        "build-essential",
        "patchelf",
        "COPY builder-requirements.lock",
        "COPY builder_verify.py",
    ):
        assert value in builder
    for value in (
        "--network=none",
        "--read-only",
        "--userns=keep-id",
        "--cap-drop=ALL",
        "no-new-privileges",
        "--mount",
        "destination=/build",
    ):
        assert value in script or value in (Path(__file__).with_name("build.py").read_text())
    assert "--require-hashes" in builder and "auditwheel" in requirements
    assert "io.dimos.sdist-sha256" in text
    assert "io.dimos.base-image" in text
    for tool in (
        "bash",
        "coreutils",
        "file",
        "findutils",
        "git",
        "grep",
        "gzip",
        "jq",
        "procps",
        "sed",
        "tar",
    ):
        assert tool in text
    assert "tmpfs-size=" in Path(__file__).with_name("build.py").read_text()
