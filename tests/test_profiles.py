import importlib.util
from pathlib import Path

import pytest
import yaml

SOURCE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("qb2_profile", SOURCE / "scripts/qb2_profile.py")
profiles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiles)


def test_shipped_profiles_and_explicit_apply():
    config = profiles.load_profiles(SOURCE / "config/profiles.yaml")
    assert {name: value["target_frame_rate"] for name, value in config.items()} == {
        "balanced": 4.0, "motion": 6.0, "mapping": 2.0}
    for name in config:
        options = profiles.parser().parse_args([name])
        request = profiles.make_request(config[name]["target_frame_rate"], options.apply)
        assert request.dry_run
        assert request.fields == ["target_frame_rate"]
        assert request.configuration.target_frame_rate_enabled
        options = profiles.parser().parse_args([name, "--apply"])
        request = profiles.make_request(config[name]["target_frame_rate"], options.apply)
        assert not request.dry_run
        assert request.configuration.target_frame_rate == config[name]["target_frame_rate"]


@pytest.mark.parametrize("rate", [0, -1, float("nan"), float("inf"), True, "6 Hz"])
def test_invalid_profile_rates_fail_before_rpc(tmp_path, rate):
    config = profiles.load_profiles(SOURCE / "config/profiles.yaml")
    config["motion"]["target_frame_rate"] = rate
    path = tmp_path / "profiles.yaml"
    path.write_text(yaml.safe_dump({"profiles": config}))
    with pytest.raises(ValueError, match="finite and positive"):
        profiles.load_profiles(path)


def test_unknown_field_is_not_silently_ignored(tmp_path):
    config = profiles.load_profiles(SOURCE / "config/profiles.yaml")
    config["mapping"]["maximum_range"] = 2.0
    path = tmp_path / "profiles.yaml"
    path.write_text(yaml.safe_dump({"profiles": config}))
    with pytest.raises(ValueError, match="Unknown setting"):
        profiles.load_profiles(path)


def test_incomplete_profiles_fail(tmp_path):
    path = tmp_path / "profiles.yaml"
    path.write_text("profiles: {balanced: {target_frame_rate: 4.0}}")
    with pytest.raises(ValueError, match="balanced, motion, and mapping"):
        profiles.load_profiles(path)
