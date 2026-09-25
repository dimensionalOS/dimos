import unittest

from pydantic import ValidationError

from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig, PointLioTuning


class ConfigTests(unittest.TestCase):
    def test_default_disabled(self) -> None:
        self.assertEqual(PointLioConfig().maximum_input_gap_s, 0.0)

    def test_serialization(self) -> None:
        config = PointLioConfig(
            host_ip="198.18.0.5", lidar_ip="198.18.0.155", maximum_input_gap_s=1.0
        )
        self.assertEqual(config.to_config_dict()["maximum_input_gap_s"], 1.0)

    def test_invalid(self) -> None:
        for value in [-1, float("nan"), float("inf"), 86401, 1e-12]:
            with self.subTest(value=value), self.assertRaises(ValidationError):
                PointLioConfig(maximum_input_gap_s=value)

    def test_boundaries(self) -> None:
        for value in [0.0, 1e-9, 86400.0]:
            self.assertEqual(PointLioConfig(maximum_input_gap_s=value).maximum_input_gap_s, value)

    def test_not_in_shared_rust_tuning(self) -> None:
        self.assertNotIn("maximum_input_gap_s", PointLioTuning.model_fields)


if __name__ == "__main__":
    unittest.main()
