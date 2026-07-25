from pydantic import ValidationError
import pytest

from dimos.mapping.relocalization.module import MIN_LOCAL_POINTS, Config


def test_min_local_points_preserves_official_default() -> None:
    assert Config().min_local_points == MIN_LOCAL_POINTS


def test_min_local_points_is_explicitly_configurable() -> None:
    assert Config(min_local_points=35_000).min_local_points == 35_000


def test_min_local_points_must_be_positive() -> None:
    with pytest.raises(ValidationError):
        Config(min_local_points=0)
