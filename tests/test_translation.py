"""
Unit tests for Translation class.

Tests translation vector validation, arithmetic operations,
and unit conversion methods.
"""

import unittest

import numpy as np

from se3kit import translation


class TestTranslation(unittest.TestCase):
    """Tests for the Translation class."""

    def test_translation_vector_validity(self):
        """Test validation of translation vectors."""
        vec = np.asarray([1, 2, 3])
        self.assertTrue(
            translation.Translation.is_valid(vec, verbose=False),
            "Expected vec to be a valid translation vector",
        )

        vec_bad = np.asarray([[1], [2], [3.0], [3]])
        self.assertFalse(
            translation.Translation.is_valid(vec_bad, verbose=False),
            "Expected vec_bad to be invalid (size != 3)",
        )


if __name__ == "__main__":
    unittest.main()
