"""Geometry of mrRobot's OpenFleX upper body, free of ROS and Webots.

Forward kinematics of the two 7-DOF OpenArmX arms on the lift carriage, the
position of each shoulder for a given base pose and lift height, and the
reach test the mission's docking uses. Inverse kinematics is MoveIt's job
(KDL on the URDF chains); this module exists so that "can the arm get there
from here" can be answered in a millisecond from a base pose, and so the
numbers are pinned by test/test_arm_kinematics.py against the URDF itself.

Every constant below is OpenFleX's (openarmx_description/config/arm/v10/
kinematics.yaml and the arm macro's axes) or the mount mrRobot gives it
(mrRobot.urdf.xacro). Frames: base_link is x forward, y left, z up.
"""

import math

# The Husky is built at 0.85 of Clearpath's size (mrRobot.urdf.xacro's
# base_scale). Everything from lift_base_link up is OpenFleX's and full
# size, so only the two numbers below move: the chassis sits lower and the
# column stands on a lower plate. Keep them in step with the xacro --
# test_base_geometry_matches_the_urdf pins them to it.
BASE_SCALE = 0.85
BUMPER_X = BASE_SCALE * 0.4937         # the chassis' nose ahead of base_link
HALF_WIDTH = BASE_SCALE * 0.33465      # half the chassis' width over the wheels

# --- the mount: base_link -> lift_base_link -> lift_carriage_link -> arm bases
BASE_LINK_Z = 0.112438                # base_link above the floor (map z = base z + this)
LIFT_XYZ = (0.0, 0.0, 0.3482975)         # lift_base_link in base_link
LIFT_YAW = -math.pi / 2               # the column's +y is the robot's +x
CARRIAGE_XYZ = (0.0, 0.055, 0.81)     # lift_carriage_link in lift_base_link at lift = 0
LIFT_RANGE = (-0.75, 0.40)            # lift_joint, metres about mid travel
# What the mission uses of it: below -0.50 even a tucked hand (0.7623 above
# base_link at lift 0) comes down into the chassis; the arm reaches the
# rest. Unchanged by base_scale: the plate top and the carriage both drop
# by 0.0377, so the clearance at the bottom stop is the same 4.9 cm.
LIFT_USABLE = (-0.50, 0.40)
# left_/right_link0_base in lift_carriage_link: (xyz, rpy)
ARM_BASE = {
    "left": ((-0.075, 0.14555, 0.025), (-math.pi / 2, 0.0, math.pi / 2)),
    "right": ((0.075, 0.14555, 0.025), (math.pi / 2, 0.0, math.pi / 2)),
}

# --- the arm: OpenFleX's joint table, (xyz, rpy, axis) per joint, link0 -> link7
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")


def joint_table(side):
    s = 1.0 if side == "right" else -1.0     # joint2's frame and joint7's axis mirror
    return [
        ((0.0, 0.0, 0.058), (0.0, 0.0, 0.0), (0, 0, 1)),
        ((-0.0205, 0.0, 0.081), (s * math.pi / 2, 0.0, 0.0), (-1, 0, 0)),
        ((0.02, 0.0, 0.099), (0.0, 0.0, 0.0), (0, 0, 1)),
        ((0.0, 0.031002, 0.14181), (0.0, 0.0, 0.0), (0, 1, 0)),
        ((0.0, -0.0309, 0.126), (0.0, 0.0, 0.0), (0, 0, 1)),
        ((0.037426, 0.0, 0.131), (0.0, 0.0, 0.0), (1, 0, 0)),
        ((-0.0375, 0.0, 0.0), (0.0, 0.0, 0.0), (0, s, 0)),
    ]


HAND_XYZ = (0.0, 0.0, 0.1001)         # openarmx_*_hand in link7
TCP_XYZ = (0.0, 0.0, 0.08)            # openarmx_*_hand_tcp in the hand: between the finger pads

# Per-side joint limits (joint2 mirrors; OpenFleX offsets joint1 and joint2 by
# their `kinematics_offset`, which the URDF applies to the limits).
LIMITS = {
    "left": {"joint1": (-3.344, 0.906), "joint2": (-3.271, 0.129), "joint3": (-1.57, 1.57),
             "joint4": (0.0, 1.8), "joint5": (-1.5, 1.5), "joint6": (-0.75, 0.75),
             "joint7": (-1.4, 1.4)},
    "right": {"joint1": (-1.25, 3.0), "joint2": (-0.129, 3.271), "joint3": (-1.57, 1.57),
              "joint4": (0.0, 1.8), "joint5": (-1.5, 1.5), "joint6": (-0.75, 0.75),
              "joint7": (-1.4, 1.4)},
}

# Reach: OpenFleX quotes ~0.714 m single-arm reach. Measured with fk() below
# from joint2's axis (the shoulder proper): joint2..TCP fully extended is
# 0.099 + 0.14181 + 0.126 + 0.131 + 0.1001 + 0.08 = 0.678 along the chain,
# plus the lateral offsets; the sphere below is what the reach test uses.
REACH = 0.68
MIN_REACH = 0.15
FINGER_OPEN = 0.07
FINGER_CLOSED = 0.0
# what the left hand's spring fingers are told when closing ON something:
# 45 cm past closed. The gripper controller turns the position error into
# force (200 N/m), so this is the pinch: 98 N on a 9 cm jar -- under the
# 120 N the motors cap at, so a jar pushed off centre still meets a spring
# on each side (at the cap both fingers push alike and nothing centres
# it); at 49 N a jolt through the robot popped a jar out (measured).
# Closing on nothing, they stop on their hard stop 2 cm past closed, and
# read so.
FINGER_SQUEEZE = -0.45
# ...and while the BASE moves with a jar in hand: 10 cm past closed, 29 N.
# The 98 N pinch made the whole robot turn as if its left wheels gripped
# and its right ones slid: a 40 deg turn on the spot walked the base 14-22
# cm (43 cm once, into the counter), where the same turn with the arm in
# the same pose and no jar moved it 5-9 cm; 69 N walked 8-20 cm, 29 N 2-5
# (measured against the simulator's ground truth). The jar stayed put at all of them; it
# slipped out at 9 N. Straight lines never cared.
FINGER_CARRY = -0.10
# for the door's bar, were the spring hand ever to take it: all the motors
# have (the cap). The right hand, which does, is position-controlled.
FINGER_SQUEEZE_HARD = -0.60
SIDE_SIGN = {"left": +1.0, "right": -1.0}


# ----------------------------------------------------------------- algebra
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _axis(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


class Frame:
    """A rigid transform: p' = R p + t."""

    def __init__(self, t=(0, 0, 0), R=None):
        self.t = list(t)
        self.R = R or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    def then(self, t=(0, 0, 0), rpy=(0, 0, 0), R=None):
        R = R or _rpy(*rpy)
        return Frame([self.t[i] + _apply(self.R, t)[i] for i in range(3)], _mul(self.R, R))

    def point(self, p):
        v = _apply(self.R, p)
        return tuple(self.t[i] + v[i] for i in range(3))


# ----------------------------------------------------------------- chains
def carriage_frame(lift=0.0):
    """lift_carriage_link in base_link."""
    return (Frame().then(LIFT_XYZ, (0, 0, LIFT_YAW))
            .then((CARRIAGE_XYZ[0], CARRIAGE_XYZ[1], CARRIAGE_XYZ[2] + lift)))


def arm_base_frame(side, lift=0.0):
    """openarmx_<side>_link0 in base_link."""
    xyz, rpy = ARM_BASE[side]
    return carriage_frame(lift).then(xyz, rpy)


def fk(side, angles, lift=0.0, upto=None):
    """Position of the TCP (or of joint `upto`, 1..7) in base_link.

    `angles` is a dict keyed by ARM_JOINTS or a sequence of 7 values.
    """
    q = [angles[j] for j in ARM_JOINTS] if isinstance(angles, dict) else list(angles)
    f = arm_base_frame(side, lift)
    for i, (xyz, rpy, axis) in enumerate(joint_table(side)):
        f = f.then(xyz, rpy)
        if upto == i + 1:
            return f.point((0, 0, 0))
        f = f.then(R=_axis(axis, q[i]))
    f = f.then(HAND_XYZ)
    return f.point(TCP_XYZ)


def shoulder_in_base(side, lift=0.0):
    """joint2's axis point -- the shoulder the reach sphere is centred on."""
    return fk(side, [0.0] * 7, lift, upto=2)


def shoulder_in_world(side, base_x, base_y, base_yaw, lift=0.0):
    """The shoulder in map coordinates for a base pose (x, y, yaw in map),
    height included: map z, not base_link z."""
    sx, sy, sz = shoulder_in_base(side, lift)
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    return (base_x + c * sx - s * sy, base_y + s * sx + c * sy, sz + BASE_LINK_Z)


def world_to_shoulder(side, base_x, base_y, base_yaw, target, lift=0.0):
    """Express a world point relative to the shoulder, in base_link axes."""
    sx, sy, sz = shoulder_in_world(side, base_x, base_y, base_yaw, lift)
    ex, ey, ez = target[0] - sx, target[1] - sy, target[2] - sz
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    return (c * ex + s * ey, -s * ex + c * ey, ez)


def best_lift_for(side, base_x, base_y, base_yaw, target, shoulder_above=0.0):
    """The lift height that puts the shoulder `shoulder_above` metres above
    the target (level with it by default), clamped to the column's travel:
    the lift is the 8th degree of freedom, and it should do the vertical
    work so the arm can do the reaching. Each grip has its own window
    (KDL over a grid: a jar at shoulder level or a little below, the door
    handle's vertical-pad grip only 0.22-0.37 m below the shoulder), so
    the callers pass what their grip wants."""
    dz = target[2] + shoulder_above - shoulder_in_world(side, base_x, base_y, base_yaw, 0.0)[2]
    return max(LIFT_USABLE[0], min(LIFT_USABLE[1], dz))


def can_reach(side, base_x, base_y, base_yaw, target, margin=0.05, lift=None,
              shoulder_above=0.0):
    """Is the target inside the arm's sphere from this base pose?

    With lift=None the column is assumed free to move to best_lift_for()
    with the shoulder `shoulder_above` the target (the grip's own height:
    a place from 0.35 m up spends 0.35 of the 0.68 m on the drop, and
    checked as level it passed from a spot the set-down then had no IK
    for, measured); pass a lift height to test at a fixed one.
    """
    if lift is None:
        lift = best_lift_for(side, base_x, base_y, base_yaw, target, shoulder_above)
    d = math.hypot(*world_to_shoulder(side, base_x, base_y, base_yaw, target, lift))
    return MIN_REACH <= d <= REACH - margin


def clamp(side, angles):
    return {j: max(LIMITS[side][j][0], min(LIMITS[side][j][1], v)) for j, v in angles.items()}


# ----------------------------------------------------------------- postures
def _posture(side, **angles):
    out = {j: 0.0 for j in ARM_JOINTS}
    out.update(angles)
    return clamp(side, out)


# Named arm postures, found with fk() and checked in the tests. At all zeros
# the OpenArmX arm hangs straight down, TCP 0.68 m below the shoulder at
# (0.20, +-0.21, 0.54) in base_link -- inside the base's footprint, so nothing
# can hit a hanging hand without hitting the chassis first.
#   stow   hanging, a touch of elbow so the fingers clear the wheels. Only
#          with the lift at mid travel or above: 0.41 m lower, a hanging hand
#          is inside the chassis (MoveIt then refuses every plan, its start
#          state being in collision -- measured).
#   tuck   elbow fully bent, upper arm forward-down: the hand over the base at
#          (0.36, +-0.22, 0.80), which stays 0.39 m above the chassis at the
#          lowest lift. The pose every arm takes before the column moves, and
#          the carrying pose. (The left shoulder's range is offset, +0.906 at
#          most, which is why the tuck is not further back.)
#   carry  = tuck, holding something
#   ready  elbow fully bent, hand forward at (0.60, +-0.22, 1.02): the pose
#          to start a reach from
POSTURES = {
    "stow": {"left": _posture("left", joint4=0.2), "right": _posture("right", joint4=0.2)},
    "tuck": {"left": _posture("left", joint1=0.9, joint2=-0.05, joint4=1.8),
             "right": _posture("right", joint1=-0.9, joint2=0.05, joint4=1.8)},
    "carry": {"left": _posture("left", joint1=0.9, joint2=-0.05, joint4=1.8),
              "right": _posture("right", joint1=-0.9, joint2=0.05, joint4=1.8)},
    "ready": {"left": _posture("left", joint1=0.12, joint2=-0.35, joint3=0.11, joint4=1.8),
              "right": _posture("right", joint1=-0.12, joint2=0.35, joint3=-0.11, joint4=1.8)},
}
