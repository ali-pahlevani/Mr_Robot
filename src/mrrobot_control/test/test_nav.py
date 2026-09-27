"""The navigator's geometry, without ROS: the turn-pivot model against turns
measured in the simulator, and the odometry-frame conversions the docking
drives on."""

import math

import pytest

from mrrobot_control import nav


def test_no_turn_no_shift():
    assert nav.turn_shift(0.3, 0.0) == pytest.approx((0.0, 0.0), abs=1e-12)


def test_whole_turn_comes_back():
    assert nav.turn_shift(1.1, 2 * math.pi) == pytest.approx((0.0, 0.0), abs=1e-12)


@pytest.mark.parametrize("heading, turn, ahead, left", [
    # two errand runs: start heading, turn, and how far base_link
    # really moved (ahead, left of the start heading), from the truth
    (-0.1, -107.8, -8.0, -10.8),
    (0.3, 91.4, -9.2, 8.3),
    (98.8, -98.4, -8.5, -11.0),
    (-1.2, 91.5, -8.9, 9.5),
    (84.3, -83.6, -7.9, -7.8),
])
def test_turn_shift_matches_measured_turns(heading, turn, ahead, left):
    h = math.radians(heading)
    dx, dy = nav.turn_shift(h, math.radians(turn))
    c, s = math.cos(h), math.sin(h)
    got = (100 * (c * dx + s * dy), 100 * (-s * dx + c * dy))
    # within 2.5 cm of a 12 cm shift: what hop_to's aim can rely on
    assert got == pytest.approx((ahead, left), abs=2.5)


def _pose(x, y, yaw):
    return x, y, yaw


class _Fixed(nav.Navigator):
    """A navigator whose odometry pose is set by hand."""

    def __init__(self, odom):
        self.slip = [0.0, 0.0]
        self._odom = odom

    def pose(self, timeout=2.0, frame="map"):
        assert frame == "odom"
        return self._odom


def test_map_to_odom_inverts_the_anchor():
    anchor = (0.4, -0.2, math.radians(30))
    n = _Fixed(_pose(0.0, 0.0, 0.0))
    gx, gy, gyaw = n.map_to_odom(1.0, 0.5, math.radians(10), anchor)
    # back through the anchor: odom -> map
    tx, ty, r = anchor
    c, s = math.cos(r), math.sin(r)
    assert (tx + c * gx - s * gy, ty + s * gx + c * gy) == pytest.approx((1.0, 0.5))
    assert gyaw == pytest.approx(math.radians(-20))


def test_dead_reckoning_adds_the_slip_the_odometry_missed():
    anchor = (0.0, 0.0, 0.0)
    n = _Fixed(_pose(1.0, 2.0, 0.5))
    n.slip = [0.1, -0.05]
    assert n.dead_reckoning(anchor) == pytest.approx((1.1, 1.95, 0.5))
    # and odom_target is its inverse: driving the odometry there dead-reckons
    # to the goal
    gx, gy, gyaw = n.odom_target(1.1, 1.95, 0.5, anchor)
    assert (gx, gy, gyaw) == pytest.approx((1.0, 2.0, 0.5))


def test_steer_is_bounded():
    v, kappa = nav.steer(-0.02, 0.2, 0.0)
    assert abs(kappa) == pytest.approx(nav.KAPPA_MAX)
    assert v == pytest.approx(0.035)       # the crawl that still breaks friction


def test_slides_add_up():
    # a turn made in two parts moves base_link as far as the whole turn
    h, a, b = 0.4, -0.3, 0.9
    one = nav.turn_shift(h, a, nav.CURVE_PIVOT)
    two = nav.turn_shift(h + a, b, nav.CURVE_PIVOT)
    assert (one[0] + two[0], one[1] + two[1]) == pytest.approx(
        nav.turn_shift(h, a + b, nav.CURVE_PIVOT))


def test_curve_slide_matches_the_measured_one():
    # the approach into the microwave's cavity, five errands: the base's
    # heading went from 0 to -25.6 deg and it slid (+1.5, -6.6) cm that the
    # odometry did not see
    sx, sy = nav.turn_shift(0.0, math.radians(-25.6), nav.CURVE_PIVOT)
    assert (100 * sx, 100 * sy) == pytest.approx((1.5, -6.6), abs=0.5)


def test_straight_approach_ends_on_the_line():
    along, lateral, psi, tightest = nav.approach_path(-0.4, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert along == pytest.approx(0.0, abs=0.01)
    assert (lateral, psi, tightest) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_approach_takes_out_sideways_error():
    # 8 cm to the side with a 0.35 m run-up: the simulation that set
    # RUNUP_PER_LATERAL (within 3 mm and 4 deg; the slide the S-bend makes
    # and the last turn's take it to 5 mm)
    _, lateral, psi, _ = nav.approach_path(-0.35, 0.08, 0.0, 0.0, 0.0, 0.0)
    assert abs(lateral) < 0.008
    assert abs(math.degrees(psi)) < 1.5


def test_the_cavity_is_one_curve_from_square_on():
    # the microwave's cavity for the left hand (at 0.77 ahead, 0.36 left)
    # faced 26 deg right, from 0.37 m back from the door's dock, square on:
    # a way in mission.face_for finds, slide allowed for
    h = math.radians(-26.0)
    c, s = math.cos(h), math.sin(h)
    gx, gy = 1.60 - (c * 0.77 - s * 0.36), 0.15 - (s * 0.77 + c * 0.36)
    _, lateral, psi, tightest = nav.approach_path(0.765 - 0.12 - 0.25, 0.33, 0.0, gx, gy, h)
    assert abs(lateral) < 0.005
    assert abs(math.degrees(psi)) < 1.5
    assert tightest < 2.0
