"""Semantic isolation and sequence consistency for post-load RGB transforms."""
import unittest

import numpy as np

from training_data import AugmentationConfig, VisualAugmenter


class VisualAugmentationTests(unittest.TestCase):
    def setUp(self):
        image = np.full((8, 10, 3), (70, 110, 160), dtype=np.uint8)
        mask = np.tile(np.array([-1, -3, -2, 0, 7], dtype=np.int32), (8, 2))
        self.rgb = np.broadcast_to(image, (3, 2, *image.shape)).copy()
        self.mask = np.broadcast_to(mask, self.rgb.shape[:-1]).copy()

    def test_seeded_sequence_transform_preserves_robot_and_masks(self):
        transform = VisualAugmenter()
        original = self.rgb.copy()
        masks = self.mask.copy()
        result = transform(self.rgb, self.mask, seed=42, known_instance_ids=(7, 11))
        repeated = transform(self.rgb, self.mask, seed=42, known_instance_ids=(7, 11))
        np.testing.assert_array_equal(result.rgb, repeated.rgb)
        np.testing.assert_array_equal(result.rgb[self.mask == -1], original[self.mask == -1])
        np.testing.assert_array_equal(result.instance_id, masks)
        np.testing.assert_array_equal(self.rgb, original)
        np.testing.assert_array_equal(self.mask, masks)
        cropped_sequence = transform(self.rgb[1:2, :1], self.mask[1:2, :1], seed=42, known_instance_ids=(7,))
        np.testing.assert_array_equal(cropped_sequence.rgb[0, 0], result.rgb[1, 0])
        changed = transform(self.rgb, self.mask, seed=43, known_instance_ids=(7, 11))
        self.assertTrue(np.any(result.rgb[self.mask != -1] != changed.rgb[self.mask != -1]))

    def test_table_replacement_cannot_touch_objects_or_background(self):
        transform = VisualAugmenter(AugmentationConfig(tint_objects=False, replace_background=False))
        result = transform(self.rgb, self.mask, seed=9)
        table = self.mask == -3
        np.testing.assert_array_equal(result.rgb[~table], self.rgb[~table])
        self.assertTrue(np.any(result.rgb[table] != self.rgb[table]))

    def test_object_tint_preserves_luminance_at_gamut_boundaries(self):
        image = np.array([[[0, 0, 0], [255, 255, 255], [255, 0, 0], [0, 255, 0],
                           [0, 0, 255], [10, 30, 50], [180, 210, 240]]], dtype=np.uint8)
        mask = np.ones(image.shape[:-1], dtype=np.int32)
        transform = VisualAugmenter(AugmentationConfig(replace_table=False, replace_background=False, tint_strength=1.0))
        result = transform(image, mask, seed=71)
        weights = np.array([.2126, .7152, .0722])
        np.testing.assert_allclose(result.rgb @ weights, image @ weights, atol=.51, rtol=0)
        np.testing.assert_array_equal(result.rgb[0, :2], image[0, :2])
        self.assertTrue(np.any(result.rgb[0, 2:] != image[0, 2:]))


if __name__ == '__main__':
    unittest.main()
