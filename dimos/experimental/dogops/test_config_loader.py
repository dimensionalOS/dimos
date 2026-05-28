from dimos.experimental.dogops.config_loader import load_dogops_config


def test_load_dogops_demo_config() -> None:
    config = load_dogops_config()

    assert config.site.site_id == "dogops_demo_site"
    assert config.site.tag_family == "tag36h11"
    assert config.site.marker_length_m == 0.14
    assert config.site.package_by_id()["PKG-104"].expected_zone_id == "QA_HOLD"
    assert config.manifest.manifest_id == "inbound_manifest_demo"
    assert len(config.manifest.items) == 4
    assert config.policy.rule_for_type("blocked_cooling") is not None
    assert config.mission.mission_id == "receiving_sre_demo"
    assert "inspect_cooling" in config.mission.simulation_observations
