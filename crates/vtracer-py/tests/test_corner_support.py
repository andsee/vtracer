import unittest

import vtracer


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
        config = vtracer.Config(clustering="bw", corner_support=4.0, filter_speckle=0)
        self.assertEqual(config.corner_support, 4.0)
        self.assertIn(
            "<svg",
            config.convert_pixels(bytes([0, 0, 0, 255]), 1, 1),
        )


if __name__ == "__main__":
    unittest.main()
