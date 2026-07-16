from pathlib import Path

import pytest

from dimos.utils.config import load_config_mapping


@pytest.mark.parametrize(
    ("filename", "contents"),
    [
        ("config.yaml", "module:\n  enabled: true # deployment flag\n"),
        ("config.json", '{"module": {"enabled": true}}'),
    ],
)
def test_load_config_mapping_supports_yaml_and_json(
    tmp_path: Path, filename: str, contents: str
) -> None:
    config_path = tmp_path / filename
    config_path.write_text(contents)

    assert load_config_mapping(config_path) == {"module": {"enabled": True}}


@pytest.mark.parametrize(
    ("contents", "match"),
    [
        ("module: [", "Could not parse YAML or JSON config file"),
        ("- not\n- a mapping\n", "must contain a mapping at the top level"),
    ],
)
def test_load_config_mapping_rejects_invalid_roots(
    tmp_path: Path, contents: str, match: str
) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(contents)

    with pytest.raises(ValueError, match=match):
        load_config_mapping(config_path)


def test_load_config_mapping_allows_missing_optional_file(tmp_path: Path) -> None:
    assert load_config_mapping(tmp_path / "missing.yaml", missing_ok=True) == {}
