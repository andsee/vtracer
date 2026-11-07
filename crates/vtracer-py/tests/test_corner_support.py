import unittest
from pathlib import Path

import vtracer


def snowman_pixels():
    fixture = (
        Path(__file__).resolve().parents[3]
        / "crates"
        / "vtracer"
        / "tests"
        / "fixtures"
        / "snowman-bow.pbm"
    )
    tokens = [
        token
        for line in fixture.read_text().splitlines()
        for token in line.split("#", 1)[0].split()
    ]
    assert tokens[0] == "P1"
    width, height = map(int, tokens[1:3])
    pixels = bytearray()
    for bit in tokens[3:]:
        value = 0 if bit == "1" else 255
        pixels.extend((value, value, value, 255))
    return bytes(pixels), width, height


class CornerSupportTests(unittest.TestCase):
    def test_default_and_keyword_property(self):
        config = vtracer.Config()
        self.assertEqual(config.corner_support, 0.0)
        config.corner_support = 4.0
        self.assertEqual(config.corner_support, 4.0)

    def test_new_parameter_preserves_old_positional_arguments(self):
        config = vtracer.Config(
            "bw",
            "stacked",
            "spline",
            0,
            6,
            16,
            60,
            4.0,
            10,
            45,
            None,
            2,
            None,
            None,
            1,
            128,
            False,
            0,
            15.0,
            128,
        )
        self.assertEqual(config.watershed_detail, 128)
        self.assertEqual(config.corner_support, 0.0)
        self.assertIn("<svg", config.convert_pixels(bytes([0, 0, 0, 255]), 1, 1))

    def test_keyword_support_is_used(self):
        pixels, width, height = snowman_pixels()
        common = dict(
            clustering="bw",
            filter_speckle=0,
            optimize=0,
        )
        legacy = vtracer.Config(**common, corner_support=0.0).convert_pixels(
            pixels, width, height
        )
        config = vtracer.Config(**common, corner_support=4.0)
        self.assertEqual(config.corner_support, 4.0)
        supported = config.convert_pixels(pixels, width, height)
        self.assertNotEqual(
            legacy,
            supported,
            "positive corner support changes the fitted raster geometry",
        )
        self.assertGreater(legacy.count("C"), supported.count("C"))


if __name__ == "__main__":
    unittest.main()
