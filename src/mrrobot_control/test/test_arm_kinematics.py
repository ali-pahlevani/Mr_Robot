"""The kinematics module must be the URDF's geometry, not a copy of it that
drifted: every joint origin, axis and mount in arm_kinematics is checked
against the expanded URDF, and the postures and reach against the FK."""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

from mrrobot_control import arm_kinematics as k


def rpy_matrix(r, p, y):
    """Flattened, so pytest.approx can compare two rotations."""
    return [v for row in k._rpy(r, p, y) for v in row]


@pytest.fixture(scope="module")
def urdf():
    from ament_index_python.packages import get_package_share_directory
    xacro = os.path.join(get_package_share_directory("mrrobot_description"),
                         "urdf", "mrRobot.urdf.xacro")
    text = subprocess.check_output(["xacro", xacro, "use_webots:=false",
                                    "use_ros2_control:=false"])
    root = ET.fromstring(text)
    return {j.get("name"): j for j in root.iter("joint")}


def origin(j):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy


@pytest.mark.parametrize("side", ["left", "right"])
def test_joint_table_is_the_urdfs(urdf, side):
    for i, (xyz, rpy, axis) in enumerate(k.joint_table(side)):
        j = urdf[f"openarmx_{side}_joint{i + 1}"]
        uxyz, urpy = origin(j)
        assert uxyz == pytest.approx(list(xyz), abs=1e-5), j.get("name")
        # compare rotations as matrices: rpy representations may differ
        assert rpy_matrix(*urpy) == pytest.approx(rpy_matrix(*rpy), abs=1e-5), j.get("name")
        assert [float(v) for v in j.find("axis").get("xyz").split()] == pytest.approx(list(axis))
        lo, hi = k.LIMITS[side][f"joint{i + 1}"]
        lim = j.find("limit")
        assert float(lim.get("lower")) == pytest.approx(lo, abs=1e-3)
        assert float(lim.get("upper")) == pytest.approx(hi, abs=1e-3)


def test_base_geometry_is_the_urdfs(urdf):
    """The base is built at base_scale and several modules carry its numbers;
    this is what stops them drifting from the xacro."""
    assert origin(urdf["base_footprint_joint"])[0][2] == pytest.approx(k.BASE_LINK_Z)
    # the nose and the half width, off the chassis collision box and the wheels
    from ament_index_python.packages import get_package_share_directory
    xacro_root = ET.fromstring(subprocess.check_output(
        ["xacro", os.path.join(get_package_share_directory("mrrobot_description"),
                               "urdf", "mrRobot.urdf.xacro"),
         "use_webots:=false", "use_ros2_control:=false"]))
    base = next(x for x in xacro_root.iter("link") if x.get("name") == "base_link")
    box = base.find("collision/geometry/box")
    assert float(box.get("size").split()[0]) / 2 == pytest.approx(k.BUMPER_X, abs=1e-4)
    wheel = next(j for j in xacro_root.iter("joint")
                 if j.get("name") == "front_left_wheel_joint")
    cyl = next(x for x in xacro_root.iter("link")
               if x.get("name") == "front_left_wheel_link").find("collision/geometry/cylinder")
    half = origin(wheel)[0][1] + float(cyl.get("length")) / 2
    assert half == pytest.approx(k.HALF_WIDTH, abs=1e-4)


def test_mounts_are_the_urdfs(urdf):
    xyz, rpy = origin(urdf["lift_base_joint"])
    assert xyz == pytest.approx(list(k.LIFT_XYZ))
    assert rpy == pytest.approx([0, 0, k.LIFT_YAW])
    xyz, _ = origin(urdf["lift_joint"])
    assert xyz == pytest.approx(list(k.CARRIAGE_XYZ))
    lim = urdf["lift_joint"].find("limit")
    assert (float(lim.get("lower")), float(lim.get("upper"))) == pytest.approx(k.LIFT_RANGE)
    for side in ("left", "right"):
        xyz, rpy = origin(urdf[f"{side}_joint0_base"])
        assert xyz == pytest.approx(list(k.ARM_BASE[side][0]))
        assert rpy_matrix(*rpy) == pytest.approx(rpy_matrix(*k.ARM_BASE[side][1]), abs=1e-5)
        assert origin(urdf[f"{side}_openarmx_hand_joint"])[0] == pytest.approx(list(k.HAND_XYZ))
        assert origin(urdf[f"openarmx_{side}_hand_tcp_joint"])[0] == pytest.approx(list(k.TCP_XYZ))


def test_arms_hang_down_at_zero():
    for side in ("left", "right"):
        tcp = k.fk(side, [0.0] * 7)
        shoulder = k.shoulder_in_base(side)
        assert tcp[2] < shoulder[2] - 0.6
        assert abs(tcp[0] - shoulder[0]) < 0.05
        assert abs(tcp[1] - shoulder[1]) < 0.01


def test_reach_matches_fk():
    """A fully extended arm reaches REACH from the shoulder, and no further."""
    for side in ("left", "right"):
        s = k.shoulder_in_base(side)
        far = max(math.dist(k.fk(side, [q1, q2, 0, 0, 0, 0, 0]), s)
                  for q1 in [x / 10 for x in range(-12, 13)]
                  for q2 in [x / 10 for x in range(-12, 13)])
        assert far == pytest.approx(k.REACH, abs=0.02)


def test_postures_mirror_and_stay_in_footprint():
    for name, sides in k.POSTURES.items():
        # OpenFleX's arm is not a perfect mirror (joint4/5 carry y offsets
        # that joint2's +-pi/2 does not flip), so allow a few millimetres.
        left, right = k.fk("left", sides["left"]), k.fk("right", sides["right"])
        assert left[0] == pytest.approx(right[0], abs=5e-3), name
        assert left[1] == pytest.approx(-right[1], abs=5e-3), name
        assert left[2] == pytest.approx(right[2], abs=5e-3), name
        for side, angles in sides.items():
            for j, v in angles.items():
                lo, hi = k.LIMITS[side][j]
                assert lo <= v <= hi, (name, side, j)
        if name in ("stow", "carry", "tuck"):
            assert abs(left[0]) < k.BUMPER_X and abs(left[1]) < k.HALF_WIDTH, name
        if name in ("carry", "tuck"):
            # clear of the chassis top at the lowest lift the mission uses
            assert left[2] + k.LIFT_USABLE[0] > k.BASE_SCALE * 0.25135 + 0.04, name


def test_lift_does_the_vertical_work():
    """With the lift free, a target level with the shoulder at any height in
    the column's travel is reachable at the same horizontal distance."""
    for z in (0.75, 1.0, 1.3):
        target = (0.6, 0.0, z)
        assert k.can_reach("right", 0.0, 0.0, 0.0, target)
    assert not k.can_reach("right", 0.0, 0.0, 0.0, (0.9, 0.0, 1.0))


def test_world_to_shoulder_round_trip():
    base = (1.0, -2.0, 0.7)
    target = (1.5, -1.6, 0.9)
    d = k.world_to_shoulder("left", *base, target, lift=0.1)
    s = k.shoulder_in_world("left", *base, lift=0.1)
    c, sn = math.cos(base[2]), math.sin(base[2])
    back = (s[0] + c * d[0] - sn * d[1], s[1] + sn * d[0] + c * d[1], s[2] + d[2])
    assert back == pytest.approx(target)
