"""
Unit tests for Transformation class.

Tests 4x4 homogeneous transformation matrix validation,
composition, and conversion methods.
"""

import unittest

import numpy as np

from se3kit import transformation


class TestTransformation(unittest.TestCase):
    """Tests for the Transformation class."""

    def test_transformation_validity(self):
        """Test validation of transformation matrices."""
        # 3x3 input -> invalid transformation (expects 4x4)
        mat3 = np.asarray(
            [
                [0.8389628, 0.4465075, -0.3110828],
                [0.1087932, 0.4224873, 0.8998158],
                [0.5332030, -0.7887557, 0.3058742],
            ]
        )
        self.assertFalse(
            transformation.Transformation.is_valid(mat3, verbose=False),
            "3x3 matrix should not be a valid transformation",
        )

        # 3x4 input -> invalid
        mat3x4 = np.asarray(
            [
                [0.8389628, 0.4465075, -0.3110828, 1],
                [0.1087932, 0.4224873, 0.8998158, 2.0],
                [0.5332030, -0.7887557, 0.3058742, -3],
            ]
        )
        self.assertFalse(
            transformation.Transformation.is_valid(mat3x4, verbose=False),
            "3x4 matrix should not be a valid transformation",
        )

        # Proper 4x4 homogeneous transformation
        mat4 = np.asarray(
            [
                [0.8389628, 0.4465075, -0.3110828, 1],
                [0.1087932, 0.4224873, 0.8998158, 2.0],
                [0.5332030, -0.7887557, 0.3058742, -3],
                [0, 0, 0, 1],
            ]
        )
        self.assertTrue(
            transformation.Transformation.is_valid(mat4, verbose=False),
            "4x4 matrix should be a valid transformation",
        )


if __name__ == "__main__":
    unittest.main()
