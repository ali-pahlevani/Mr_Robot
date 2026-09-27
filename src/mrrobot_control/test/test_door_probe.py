"""The door probe on a synthetic wrist frame: a panel, the handle bar 15 mm in
front of it, the cavity a quarter metre behind, and the camera the way the
wrist carries it. No ROS, no simulator."""

import math

import numpy as np
import pytest

from mrrobot_control import door_probe as dp

W, H = 640, 480
FOV = 1.047                                  # the D405's, as build_proto.py sets it
FX = FY = (W / 2) / math.tan(FOV / 2)
CX, CY = W / 2, H / 2


def frame(panel=0.20, bar_rows=(228, 252), cavity_above=None, noise=0.0005, seed=1):
    """A depth image of the door seen square-on: `bar_rows` is the band of
    image rows the bar covers (the bar runs the full width, as the real one
    does across the patch); rows above `cavity_above` see into the cavity."""
    rng = np.random.default_rng(seed)
    d = np.full((H, W), panel, dtype=np.float32)
    d[bar_rows[0]:bar_rows[1], :] = panel - 0.015
    if cavity_above is not None:
        d[:cavity_above, :] = panel + 0.25
    d += rng.normal(0.0, noise, d.shape).astype(np.float32)
    return d


def test_panel_and_bar_are_found():
    fit = dp.panel_depth_from_patch(frame(), CX, CY)
    assert fit is not None
    assert fit.panel == pytest.approx(0.20, abs=0.001)
    assert fit.bar == pytest.approx(0.185, abs=0.001)
    assert fit.n_bar >= dp.MIN_BAR


def test_the_cavity_does_not_pull_the_panel_back():
    # the top third of the window looks past the door's edge into the cavity
    fit = dp.panel_depth_from_patch(frame(cavity_above=205), CX, CY)
    assert fit.panel == pytest.approx(0.20, abs=0.001)


def test_a_wall_without_a_bar_is_not_a_handle():
    fit = dp.panel_depth_from_patch(frame(bar_rows=(0, 0)), CX, CY)
    assert fit is not None and fit.bar is None


def test_the_fixture_bounds_what_is_believed():
    assert dp.panel_depth_from_patch(frame(), CX, CY, expect=0.20 + 0.10) is None
    assert dp.panel_depth_from_patch(frame(), CX, CY, expect=0.20 + 0.03) is not None


def test_nothing_in_view_is_nothing():
    d = np.full((H, W), np.inf, dtype=np.float32)
    assert dp.panel_depth_from_patch(d, CX, CY) is None


@pytest.mark.parametrize("rows,above", [((228, 252), 0.0), ((258, 282), -1.0)])
def test_the_bar_is_put_back_in_3d(rows, above):
    """A camera at the base_link origin looking along +x, upright (identity
    rotation): a bar centred on the principal row is at the camera's own
    height; one 30 rows lower is below it by 30 px at the bar's depth."""
    fit = dp.panel_depth_from_patch(frame(bar_rows=rows), CX, CY)
    x, y, z = dp.bar_in_base(fit, (FX, FY, CX, CY), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert x == pytest.approx(0.185, abs=0.001)
    assert y == pytest.approx(0.0, abs=0.002)
    row_off = (rows[0] + rows[1]) / 2 - CY
    assert z == pytest.approx(-row_off * 0.185 / FY, abs=0.001)
    assert math.copysign(1.0, z) == math.copysign(1.0, above) or abs(z) < 0.001


def test_a_camera_rolled_over_still_puts_the_bar_right():
    """The wrist can hold the door grip either way round (door()'s two rolls):
    rolled 180 deg about its view axis the image is upside down, and the bar
    below the principal row is then ABOVE the camera."""
    fit = dp.panel_depth_from_patch(frame(bar_rows=(258, 282)), CX, CY)
    rolled = (1.0, 0.0, 0.0, 0.0)            # 180 deg about x, the view axis
    _, _, z = dp.bar_in_base(fit, (FX, FY, CX, CY), (0.0, 0.0, 0.0), rolled)
    assert z > 0.005
