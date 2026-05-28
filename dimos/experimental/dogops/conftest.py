import pytest

from dimos.experimental.dogops.config_loader import load_site_config


@pytest.fixture
def dogops_site():
    return load_site_config()
