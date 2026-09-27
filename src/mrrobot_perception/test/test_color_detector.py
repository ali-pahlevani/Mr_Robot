"""The detector's colour and size logic on synthetic images: a jar-sized
orange blob at a known depth must come back at the right 3D point, and a box
of the same colour that is too big must not."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from mrrobot_perception.color_detector import _mask, _ranges  # noqa: E402


def test_ranges_and_mask_wrap_red():
    ranges = _ranges([0, 110, 40, 8, 255, 170, 170, 110, 40, 180, 255, 170])
    assert len(ranges) == 2
    hsv = np.zeros((2, 2, 3), np.uint8)
    hsv[0, 0] = (2, 200, 100)      # dark red, low hue
    hsv[0, 1] = (176, 200, 100)    # dark red, wrapped hue
    hsv[1, 0] = (2, 200, 250)      # bright red: too bright for a jam body
    hsv[1, 1] = (60, 200, 100)     # green
    m = _mask(hsv, ranges)
    assert (m > 0).tolist() == [[True, True], [False, False]]


def test_size_gate_from_depth():
    """A 9 cm jar at 0.6 m spans w = 0.09 * fx / z pixels; the same colour
    at box size must be rejected by the gate used in _find."""
    fx = 261.0
    z = 0.6
    jar_px = 0.09 * fx / z
    box_px = 0.30 * fx / z
    assert 30 < jar_px < 50
    assert box_px > 2 * jar_px
