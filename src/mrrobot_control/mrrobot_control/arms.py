"""The arms, hands, lift and head, on MoveIt 2 and the trajectory controllers.

Every motion here waits for a RESULT, never a timer: arm moves through
pymoveit2's execute_trajectory action, lift / head / gripper moves through the
controllers' FollowJointTrajectory actions, and every planning-scene change
through a re-read of the scene until the change shows up. RainBot's
pick_and_place.py documents what tuned sleeps cost it ("0.25 s was not
enough"); there are none on the motion paths here.

Geometry (all goals in base_link; the mission converts world points with the
live map->base_link pose right before each move):

  * The OpenArmX hand's approach axis is its +z, the fingers close along its
    y, and the TCP is 0.08 m out along z, between the finger pads.
  * Jars are taken with a SIDE grasp: the hand pointing +x (at the counter),
    pads closing across y. The lift puts the shoulder level with the jar, so
    the arm reaches horizontally.
  * The microwave door is worked by gripping its handle bar (a horizontal
    bar along y, 10 x 8 mm) with the pads closing across z, then dragging the
    TCP round the hinge's arc while the hand rotates with the door.
  * Grasp verification: the pads meet at the hand's centre, so a closed
    empty hand reads ~0.000 on the finger joints; a 90 mm jar stops them near
    0.045, the handle bar near 0.004. `closed_on_something` reads the joint.
"""

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mrrobot_control import arm_kinematics as k
from mrrobot_control import door_probe
from pymoveit2 import MoveIt2

# outcomes of pick / place / door
OK, MISSED, FAIL = "ok", "missed", "fail"

# Hand orientations in base_link (x, y, z, w). See the module docstring.
# The wrist camera housing sits on the hand's +x side, 7 cm out, so the roll
# about the approach axis is chosen to put hand +x UP: with it down the
# housing hung below the pads and met the worktop (measured, and it pushed
# the wrist 1.6 rad past its limit).
GRASP_FORWARD = (0.7071068, 0.0, 0.7071068, 0.0)      # hand z -> +x, hand x -> +z, pads across y
DOOR_GRIP = (0.5, 0.5, 0.5, 0.5)                       # hand z -> +x, hand x -> +y, pads across z
# Orientation tolerances about the hand's own (x, y, z): 0.25 rad each. The
# roll about the approach axis (z) is deliberately NOT free even though a jar
# is round: the wrist camera has to stay on top (above).
ORIENT_TOL = (0.25, 0.25, 0.35)
FINGER_JAR_MIN = 0.015     # finger joint reading below this after closing = empty
CARRY_GRIP_MIN = 0.042     # below this a jar is not pinched lightly for the drive (see hold)
VIA_ABOVE = 0.12           # the climb before a pre-grasp / pre-place (see pick)
PICK_HOVER = 0.08          # the one-motion reach stops this far above the pre-grasp (see pick)
# The column height a place into the microwave plans its hover at (the
# mission's `hover_lift`, see reach_out): 25 cm above mid travel. From
# the carry pose a jar hangs with its bottom at 0.85 m, under the open
# microwave door's tip and the cavity's mouth, and the swing up to the
# hover, however it was planned, dragged it through one or the other
# (measured, four runs). With the column up here first the carry pose
# itself is above both and the swing has nothing under it. The column
# then comes down with the arm out, as before. Elsewhere (a worktop, the
# table) mid travel does, and the column has less far to come down.
HOVER_LIFT = 0.25
# Where on a jar the pads take it: at its middle. The pads are 2.5 cm tall
# on an 11.5 cm jar and the jar pivots between them about the pads'
# normal, so it keeps whatever angle it is held at only by friction --
# which is enough while the hand stays level (see CARRY_TCP). Taking it
# 2.5 cm above its middle so that it hung like a pendulum was tried: a
# planned swing that turns the hand over on the way let it swing right
# round, and it arrived upside down (measured, once).
GRIP_ABOVE_MID = 0.0
HOVER_MIN_X = 0.64         # the nearest a hover is planned, in base_link (see reach_out)
# Where a jar is carried between stations, in base_link, with the column at
# mid travel: in front of the shoulder, the hand LEVEL (the forward grip).
# The folded tuck pitched the hand 40 deg, the jar pivoted in the pads to
# hang from it, and at the next station it was still tilted 40-50 deg, its
# bottom 7 cm from where the planner had it -- over the microwave's top
# instead of at its mouth (measured, four runs). Level, it stays level:
# the pads' friction holds whatever angle the jar was picked at, which
# was upright. The hand is inside the base's footprint here (the bumper
# is 0.52 m out) and the jar rides over the chassis.
CARRY_TCP = (0.32, 0.22, 0.86)
# Closed across the middle of the 8 mm bar the right fingers read
# 0.005-0.006 (measured on the bar, and in every grip since); with only the pads'
# last millimetres on it, 0.0016, and closed on nothing, 0.0001. The old
# floor of -0.005 passed both of those as grips: one of them opened the door
# by the tips of the pads, which is what looked like grabbing it from a
# distance, and the other pulled on air three times over.
FINGER_BAR_MIN = 0.003
# closed to more than this on the 8 mm bar (which grips at 0.003-0.005) =
# the fingertips are pressed on the door's face and the fingers jammed
# there (measured: 0.037 and 0.069, and the pull did nothing)
FINGER_BAR_MAX = 0.020
# Measured off Oven.proto with the microwave's [0.5, 1.2, 0.4] scale
# applied: the handle bar's box spans door-local x 0.005..0.015 and the door
# panel's spans -0.025..0, so the bar is 10 mm thick, 8 mm tall, and stands
# 5 mm clear of the panel's face. (The 2 cm square that used to be written
# here was the UNSCALED proto, which is how DOOR_GRIP_SHORT came to be tuned
# to a value that jams.) The fingertips reach 15.42 mm past the TCP -- the
# finger mesh ends at hand z 0.09542 and the TCP is at 0.08 -- so a TCP
# `short` behind the bar's centre puts the tips at 0.01542 - short past it
# and short - 0.00542 clear of the panel.
TIP_PAST_TCP = 0.01542
DOOR_BAR_PROUD = 0.005     # panel face -> the bar's far face
DOOR_BAR_THICK = 0.010
# How close the fingertips are flown to the panel. Measured directly
# (the hand driven onto the bar at four offsets with the
# fingertips read in world coordinates against the supervisor's true base
# pose): at 14.1 mm of air the fingers close to 0.0016 -- the bar caught on
# the pads' very ends -- and at 7.1 and 3.1 mm they close to 0.0051, a pinch
# across the middle of the bar. At -0.9 mm, tips just inside the panel, they
# still closed; a few millimetres further in they stop dead at 0.039, which
# is the jam. So the band that grips is wide, 2 mm of air is NOT the middle
# of it, and 7 mm is: it leaves the depth fit, the TF chain and the arm
# their few millimetres in either direction and still swallows the bar.
DOOR_PANEL_AIR = 0.007
# The ideal grip -- the bar in the middle of the 25 mm pad -- would need the
# tips 2.5 mm INSIDE the panel, so the best there is is bounded by the panel:
# air + TIP_PAST_TCP - (PROUD + THICK/2) = 0.00742, which swallows the bar
# with 2.9 mm of pad past its far face. That leaves 2 mm of error budget, so
# it is only used when the panel has been MEASURED (see measure_door_short).
# Blind, the tips keep the old 4.6 mm of air: 1-3 cm of AMCL error against a
# 2 mm budget would jam on the face, which is exactly what used to happen on
# the first go of every run.
DOOR_GRIP_SHORT = 0.010
DOOR_GRIP_SHORT_SEEN = DOOR_PANEL_AIR + TIP_PAST_TCP - (DOOR_BAR_PROUD + DOOR_BAR_THICK / 2)
# What a measured grip may ask for: 0.008 still leaves the tips clear of the
# panel. The wrist sees the panel 1.2 cm nearer than the fixture and the pose
# put it, run after run, so the measured grip is 2.1-2.6 cm short of the
# fixture's bar; capped at 2.0 (as it was), the tips were flown 1.4-2.9 mm
# from the panel instead of 7, and one pull in a dozen jammed, yanked the
# base and threw the honey out of the other hand (measured).
DOOR_SHORT_RANGE = (0.008, 0.030)
DOOR_PRE_BACK = 0.12       # the straight approach's length, back along the axis
# per finger, a 9 cm gap round the bar. Fully open (0.07), the upper
# finger ran into the cavity's ceiling slab (1.5 cm over the handle) in
# the planning scene and the straight approach stopped short of the bar;
# at 0.03 the gap was 6 cm less the centimetre the spring fingers sag
# under their own weight (they close along z here), and a hand 2 cm low
# on the bar shut above it one time in two (measured).
DOOR_FINGER_OPEN = 0.045
# the approach pose is planned to the exact grip (0.1 rad), not the 0.25
# of a jar: the straight move that follows would otherwise have to undo
# the tilt as well, and ran out of IK part way one time in three
DOOR_PRE_TOL = (0.1, 0.1, 0.1)
# Speeds (MoveIt's scaling of the joint limits): free space, a straight
# move near things, and a hand about to touch something light -- at full
# speed the first contact with the microwave door flicked it 40 degrees
# into the air (measured).
FREE_SPEED = 0.4           # 0.5 outran the wrist's path tolerance (see the controllers' yaml)
CART_SPEED = 0.5           # straight moves are short; half the joints' limits
# a planned swing with a jar in hand: the arm trails its plan by ~80 ms
# of its speed, and at 0.3 the swing from the carry pose up to the hover
# over the open microwave door, planned to clear the door, dipped 10 cm
# under its plan and the jar's rim caught the door's tip (measured, three
# times in five); at half that the dip is 5 cm and the plan's margin holds
HOLD_SPEED = 0.15
SLOW_SPEED = 0.075
# The pull that opens the door. It was SLOW_SPEED, where five steps of the
# arc took 17 s of a 44 s door; the door is 0.6 kg with its centre of mass
# over the hinge and needs 0.3 N at the handle to start, so the care was for
# the FIRST contact, which the grip has already made by then. Not faster than
# this: at 0.18 the hand outran the door's own fall, let go with the door
# still up at 34 degrees, and the door swung down through the arm as it
# tucked -- which left joint 7 outside its bounds and every plan after it
# refusing the start state (measured).
DOOR_PULL_SPEED = 0.12
# the push that shuts the door: a third of the joints' limits. (Until the
# straight moves were retimed it ran at their limits and shut the door
# every time; at SLOW_SPEED the fifteen steps took 40 s.)
DOOR_PUSH_SPEED = 0.3


def retime(traj, factor):
    """Stretch a joint trajectory in time by `factor` (velocities and
    accelerations scaled to match)."""
    for pt in traj.points:
        t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * factor
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        pt.velocities = [v / factor for v in pt.velocities]
        pt.accelerations = [a / (factor * factor) for a in pt.accelerations]


def reversed_path(traj, start=None):
    """The same joint path run backwards, in the same time: the way a hand
    went in, out. `start` (the arm's joint positions now) replaces the
    first point, so the move begins exactly where the arm stands and not
    where the forward move ended up to its tracking error."""
    def secs(pt):
        return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
    out = JointTrajectory()
    out.joint_names = list(traj.joint_names)
    total = secs(traj.points[-1])
    for pt in reversed(traj.points):
        q = JointTrajectoryPoint()
        q.positions = list(pt.positions)
        q.velocities = [-v for v in pt.velocities]
        q.accelerations = list(pt.accelerations)
        t = total - secs(pt)
        q.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        out.points.append(q)
    if start is not None:
        out.points[0].positions = [float(v) for v in start]
    return out


def arm_joints(side):
    return [f"openarmx_{side}_{j}" for j in k.ARM_JOINTS]


def finger_joints(side):
    return [f"openarmx_{side}_finger_joint1", f"openarmx_{side}_finger_joint2"]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_about_y(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


# ---------------------------------------------------------------- the door
# The microwave door drops (hinge along y at its bottom edge): a point on it
# `arm` from the hinge, with the door open by `a`, swings towards the robot
# (-x) and down. Same frame as the inputs (base_link or map, both have the
# hinge axis along y at every station).
def door_point(hinge, arm, a):
    c, s = math.cos(a), math.sin(a)
    return (hinge[0] + arm[0] * c - arm[2] * s, hinge[1] + arm[1],
            hinge[2] + arm[0] * s + arm[2] * c)


def door_handle_at(handle, hinge, a):
    return door_point(hinge, tuple(h - g for h, g in zip(handle, hinge)), a)


# The head camera in head_base_link (object_memory.py measures the same):
# CAMERA_UP above the pitch axis, CAMERA_FORWARD in front of it; and the
# neck's range (mrRobot_head.xacro).
HEAD_CAMERA_UP, HEAD_CAMERA_FORWARD = 0.16, 0.08
HEAD_PITCH, HEAD_YAW = (-1.086, 0.403), (-1.5708, 1.5708)

DOOR_OPEN = 1.5            # the Oven PROTO's stop: the dropped door lies flat
# The vertical-pad grip on the bar has IK only with the shoulder well above
# it (KDL over a grid: nothing within 0.2 m of shoulder level, everything
# 0.22-0.37 m below it at 0.55-0.66 m ahead), so the column goes UP for the
# door, unlike a jar, which is taken at shoulder level.
DOOR_SHOULDER_ABOVE = 0.35
# A jar is taken with the shoulder a little above it (the pads reach the
# jar's waist with the elbow still bent; measured over many picks).
JAR_SHOULDER_ABOVE = 0.13
# Closing: the handle bar hangs 1 cm UNDER the flat door, so it cannot be
# pinched, and balancing it on a finger lost it within a few centimetres
# of pose error (measured, many ways). The door is pushed by its FACE
# instead -- a 46 x 18 cm plane that faces down when the door is open and
# towards the robot when it is shut -- with the closed fist, pointing at
# the door as for a jar but pitched up 15 deg. The fist's top is the
# fingers' 6 cm side, which tapers from 3 cm above the TCP at the root to
# 1.1 cm at the tip (collision mesh sliced); pitched up 15 deg that taper
# is level, a 7 x 5 cm flat that lifts the face while the door lies down,
# and once it stands up the fingertips press it home. Nothing on the
# hand reaches past the tips (the palm's rails point sideways), so the
# door face is never hit by anything but the pusher. The tracked point is
# the tip's top corner: it rides 1 cm inside the face at a fixed point of
# the door 6.5 cm up from the hinge -- low enough that the whole fist
# stays hinge-ward of the bar, and it still clears the worktop by 2 cm
# under the flat door.
FIST_UP = quat_mul(quat_about_y(-0.26), GRASP_FORWARD)
# the tracked point on the door: from the hinge, in the closed door's
# frame (the face is 2.5 cm in front of the hinge; 1 cm inside it)
DOOR_PUSH_CORNER = (-0.015, 0.0, 0.065)
# TCP relative to the tip's top corner (0.5 cm ahead, 1.1 cm up, pitched)
DOOR_PUSH_OFFSET = (-0.002, 0.0, -0.012)
# The fist cannot go in at the face's height: the door's edge and the
# bar hang there. It goes in this much lower and comes up. 0.02 put the
# flat 2-8 mm under the bar and the approach was stopped by it one time
# in two (measured: the wrist 0.4 rad off its path half a second in);
# 0.03 keeps the fingertips 7 mm over the worktop slab as modelled, and
# the approach tries the other two if it is stopped.
DOOR_PUSH_DIP = 0.03
DOOR_PUSH_DIPS = (0.03, 0.02, 0.04)
# The lift as for a jar (the same grip); the column then climbs with the
# arc.
DOOR_CLOSE_SHOULDER_ABOVE = JAR_SHOULDER_ABOVE
# The push goes on PAST closed: the door's centre of mass sits right over
# the hinge, so it falls open again from anything over 3 deg (measured: 6
# and 8 deg both fell); the last steps press it home against the frame.
DOOR_CLOSED_ENOUGH = -0.30


class Arms:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.moveit = {}
        for side in ("left", "right"):
            self.moveit[side] = MoveIt2(
                node=node, joint_names=arm_joints(side), base_link_name="base_link",
                end_effector_name=f"openarmx_{side}_hand_tcp", group_name=f"{side}_arm")
            self.moveit[side].max_velocity = FREE_SPEED
            self.moveit[side].max_acceleration = FREE_SPEED
            # the tuck -> over-the-counter plans are narrow-passage problems
            # for RRTConnect: 5 s found a path two times in three (measured)
            self.moveit[side].allowed_planning_time = 12.0
            # 10 attempts, each a full RRTConnect solve whose best is kept,
            # put 9 s on the table place's fallback alone; 5 halves that and
            # the paths that come back still execute inside the controllers'
            # path tolerance (measured over the errand).
            self.moveit[side].num_planning_attempts = 5
        self._clients = {name: ActionClient(node, FollowJointTrajectory,
                                            f"/{name}_controller/follow_joint_trajectory")
                         for name in ("lift", "head", "left_gripper", "right_gripper")}
        self.holding = {"left": None, "right": None}     # attached object id per hand
        # optional callable(tag) for diagnostics (the mission's contacts log)
        self.probe = None
        import tf2_ros
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, node)
        # the right hand is the one that works the door; a second 5 Hz depth
        # subscription for a hand that never does would be for nothing
        self._wrist = door_probe.WristDepth(node, "right")

    def tcp_position(self, side):
        """The TCP's current position in base_link, or None."""
        import tf2_ros
        try:
            t = self._tf.lookup_transform("base_link", f"openarmx_{side}_hand_tcp",
                                          rclpy.time.Time()).transform.translation
            return (t.x, t.y, t.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def wait_still(self, side, timeout=1.5):
        """Until the column and the side's arm have stopped (every joint under
        1 cm/s or 0.02 rad/s), or `timeout` seconds."""
        names = ["lift_joint"] + arm_joints(side)
        end = time.time() + timeout
        while time.time() < end:
            js = self.moveit[side].joint_state
            if js is not None and js.velocity:
                vel = dict(zip(js.name, js.velocity))
                limit = {n: 0.01 if n == "lift_joint" else 0.02 for n in names}
                if all(abs(vel.get(n, 0.0)) < limit[n] for n in names):
                    return True
            self._spin(0.05)
        return False

    def measure_door_short(self, side, handle):
        """(short, dz): the `short` that puts the pads round the bar, and how
        far the bar really is above the fixture's height, read off the wrist.

        Called with the hand at `pre`, 12 cm back and pointing at the door.
        The wrist camera looks along the hand's approach axis from the TCP's
        own height, so the depth it reports IS the distance the grip cares
        about -- no lever arm, and none of the map-frame error that the
        fixture carries. The bar's own pixels, put back into 3-D through the
        camera's pose, give its height the same way: centred between the
        pads it is a grip that looks like one. None when it cannot see a
        door, and the caller then grips open-loop.
        """
        import tf2_ros
        # still first: arm and column arrive together (reach), and the
        # column can still be settling when the arm reports it is there --
        # a frame from then, put through the pose TF had a moment later,
        # read the bar 1.5 cm high and the pads closed on its edge (the bar
        # slipped out at 6 deg and the door fell back shut; measured)
        if not self.wait_still(side):
            self.log.warn("door: the arm did not come to rest; measuring anyway")
        now = self.node.get_clock().now().nanoseconds * 1e-9
        frame = None
        for _ in range(12):                 # the camera runs at 5 Hz
            self._spin(0.1)
            frame = self._wrist.latest(after=now)
            if frame is not None:
                break
        if frame is None:
            self.log.warn("door: no wrist depth frame")
            return None
        depth, (fx, fy, cx, cy) = frame
        # the arm is still (above), so the latest transforms are the frame's:
        # looked up at the frame's own stamp instead, with no spinning while
        # waiting for it, every lookup timed out and every grip went blind
        # (measured, three runs)
        try:
            cam_tf = self._tf.lookup_transform("base_link", f"{side}_wrist_camera_link",
                                               rclpy.time.Time()).transform
            cam = cam_tf.translation
            hand = self._tf.lookup_transform("base_link", f"openarmx_{side}_hand",
                                             rclpy.time.Time()).transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.log.warn(f"door: no transform for the wrist camera ({e})")
            return None
        tcp = self.tcp_position(side)
        if tcp is None:
            return None
        ahead = door_probe.norm(door_probe.quat_rotate(
            (hand.x, hand.y, hand.z, hand.w), (0.0, 0.0, 1.0)))
        cam = (cam.x, cam.y, cam.z)
        # where the fixture says the panel is, as a sanity bound
        panel_x = handle[0] + DOOR_BAR_THICK / 2 + DOOR_BAR_PROUD
        expect = sum((p - c) * a for p, c, a in
                     zip((panel_x, handle[1], handle[2]), cam, ahead))
        fit = door_probe.panel_depth_from_patch(depth, cx, cy, expect=expect)
        if fit is None:
            self.log.warn("door: the wrist is not looking at a door face")
            return None

        # TCP -> panel along the approach axis, and how much further the TCP
        # must travel for the fingertips to stop DOOR_PANEL_AIR short of it
        behind = sum((t - c) * a for t, c, a in zip(tcp, cam, ahead))
        reach = fit.panel - behind
        advance = reach - (DOOR_PANEL_AIR + TIP_PAST_TCP)
        short = handle[0] - (tcp[0] + advance * ahead[0])
        lo, hi = DOOR_SHORT_RANGE
        if fit.bar is None:
            # without the bar confirmed in front of the panel the 2 mm of air
            # is not earned: it might be a side wall or an already-open door
            lo = max(lo, DOOR_GRIP_SHORT)
            self.log.warn("door: panel seen but no handle bar in it")
        clamped = max(lo, min(hi, short))
        # the bar's height, from its own pixels; more than 3 cm off the
        # fixture is not a bar we believe (the pads span 8.6 cm open)
        dz = 0.0
        bar = door_probe.bar_in_base(fit, (fx, fy, cx, cy), cam,
                                     (cam_tf.rotation.x, cam_tf.rotation.y,
                                      cam_tf.rotation.z, cam_tf.rotation.w))
        if bar is not None and abs(bar[2] - handle[2]) < 0.03:
            dz = bar[2] - handle[2]
        self.log.info(
            f"door: wrist sees the panel {fit.panel:.4f} m out "
            f"({fit.n_panel} px, bar {fit.n_bar} px), fixture said "
            f"{expect:.4f}; grip short {clamped:.4f}"
            + ("" if clamped == short else f" (from {short:.4f})")
            + ("" if bar is None else
               f"; bar at z {bar[2]:.4f} against {handle[2]:.4f}, grip {dz:+.4f}"))
        return clamped, dz

    def tcp_quat(self, side):
        """The hand's current orientation in base_link, for cartesian moves
        that should not also be asked to twist the wrist."""
        import tf2_ros
        try:
            q = self._tf.lookup_transform("base_link", f"openarmx_{side}_hand_tcp",
                                          rclpy.time.Time()).transform.rotation
            return (q.x, q.y, q.z, q.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    # ------------------------------------------------------------ plumbing
    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _joint(self, name):
        js = self.moveit["left"].joint_state
        if js is None:
            return None
        try:
            return js.position[list(js.name).index(name)]
        except ValueError:
            return None

    def _follow(self, name, joints, positions, seconds, timeout=30.0, first=None):
        """Send one trajectory point to a controller and wait for its result.
        `first`: (positions, seconds) of a point to pass through before it."""
        client = self._clients[name]
        if not client.wait_for_server(timeout_sec=10.0):
            self.log.error(f"{name}_controller: no action server")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        for pos, t in ([first] if first else []) + [(positions, seconds)]:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.log.error(f"{name}_controller rejected the goal")
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout)
        if not result.done():
            self.log.error(f"{name}_controller: no result after {timeout:.0f} s")
            return False
        code = result.result().result.error_code
        if code != 0:
            self.log.warn(f"{name}_controller finished with error {code}: "
                          f"{result.result().result.error_string}")
        return code == 0

    def _send(self, name, joints, positions, seconds):
        """Send one trajectory point and do NOT wait: for the head's gaze, which
        is re-aimed several times a second while the base moves, and for a
        gripper that can open while the arm is already on its way."""
        client = self._clients[name]
        if not client.server_is_ready():
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        client.send_goal_async(goal)
        return True

    def head_angles_for(self, point_map):
        """(pitch, yaw) that points the head camera at a map point, or None
        without TF (the object memory's own aim, see object_memory._aim)."""
        import tf2_ros
        try:
            t = self._tf.lookup_transform("head_base_link", "map", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q, tr = t.transform.rotation, t.transform.translation
        x, y, z = point_map
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        ix, iy, iz, iw = (qw * x + qy * z - qz * y, qw * y + qz * x - qx * z,
                          qw * z + qx * y - qy * x, -qx * x - qy * y - qz * z)
        px = ix * qw + iw * -qx + iy * -qz - iz * -qy + tr.x
        py = iy * qw + iw * -qy + iz * -qx - ix * -qz + tr.y
        pz = iz * qw + iw * -qz + ix * -qy - iy * -qx + tr.z
        yaw = math.atan2(-px, py)
        pitch = math.atan2(pz - HEAD_CAMERA_UP, math.hypot(px, py) - HEAD_CAMERA_FORWARD)
        return (max(HEAD_PITCH[0], min(HEAD_PITCH[1], pitch)),
                max(HEAD_YAW[0], min(HEAD_YAW[1], yaw)))

    def gaze(self, point_map):
        """Point the head at a map point without waiting for it to get there;
        cheap to call often -- a command only goes out when the aim has moved
        by more than a degree and a half."""
        aim = self.head_angles_for(point_map)
        if aim is None:
            return
        last = getattr(self, "_gaze_sent", None)
        if last is not None and max(abs(a - b) for a, b in zip(aim, last)) < math.radians(1.5):
            return
        if self._send("head", ["openarmx_head_pitch_joint", "openarmx_head_yaw_joint"],
                      aim, 0.4):
            self._gaze_sent = aim

    # ------------------------------------------------------------ simple moves
    def lift_to(self, height):
        height = max(k.LIFT_RANGE[0], min(k.LIFT_RANGE[1], height))
        now = self._joint("lift_joint")
        if now is not None and abs(height - now) < 0.005:
            return True
        seconds = 1.0 + abs(height - (now or 0.0)) / 0.08     # 0.1 m/s is the joint's limit
        return self._follow("lift", ["lift_joint"], [height], seconds, timeout=seconds + 10)

    def look(self, pitch, yaw=0.0):
        return self._follow("head", ["openarmx_head_pitch_joint", "openarmx_head_yaw_joint"],
                            [pitch, yaw], 1.5)

    def gripper(self, side, opening):
        getattr(self, "_pinch", {}).pop(side, None)
        now = [self._joint(j) for j in finger_joints(side)]
        if None not in now and all(abs(v - opening) < 0.003 for v in now):
            return True
        return self._follow(f"{side}_gripper", finger_joints(side), [opening, opening], 1.2)

    def open_while_moving(self, side, opening):
        """Start the fingers opening and carry on: they are open long before
        a reach arrives (1.2 s against three or more)."""
        getattr(self, "_pinch", {}).pop(side, None)
        self._send(f"{side}_gripper", finger_joints(side), [opening, opening], 1.2)

    def grasp(self, side, squeeze=k.FINGER_SQUEEZE):
        """Close on whatever is between the pads, then squeeze. Two goals:
        the fingers are springs (see the controllers' yaml), and a ramp
        straight to the squeeze target passes the closing point in a tenth
        of a second -- before the arm has settled on the handle, and the
        fingers shut on nothing four times in four (measured). Closed over
        a second first, they meet the bar with the arm at rest."""
        self._spin(0.4)                       # the arm's last centimetre
        if not self.gripper(side, k.FINGER_CLOSED):
            return False
        if side == "right":                   # position-controlled: closed is the squeeze
            return True
        self._pinch = {**getattr(self, "_pinch", {}), side: squeeze}
        return self._follow(f"{side}_gripper", finger_joints(side), [squeeze, squeeze], 0.5,
                            first=([k.FINGER_CLOSED, k.FINGER_CLOSED], 0.02))

    def hold(self, side, firm):
        """The pinch on what the hand holds: firm (FINGER_SQUEEZE) whenever
        the arm works, light (FINGER_CARRY) while the base drives -- see
        arm_kinematics. Only the spring hand has a pinch to set."""
        if side != "left" or not self.holding[side]:
            return True
        want = k.FINGER_SQUEEZE if firm else k.FINGER_CARRY
        if not firm:
            # a jar gripped nearer its edge than its middle -- the pads
            # closer together than the 0.0425-0.0445 of a centred grip --
            # keeps the firm pinch: one such went to the floor on the light
            # one (measured, the pads at 0.0413)
            pos = [self._joint(j) for j in finger_joints(side)]
            if None not in pos and sum(pos) / 2.0 < CARRY_GRIP_MIN:
                self.log.info(f"{side} hand: a marginal grip ({sum(pos) / 2:.4f}); "
                              "keeping the firm pinch")
                want = k.FINGER_SQUEEZE
        was = getattr(self, "_pinch", {}).get(side)
        if was == want:
            return True
        # A new trajectory starts from where the fingers ARE, and for these
        # spring fingers that is zero pinch: the force fell to nothing for a
        # moment at every change, and once a jar went to the floor with it
        # (measured). So the old pinch first, within a control step, then
        # the ramp to the new one.
        first = None if was is None else ([was, was], 0.02)
        ok = self._follow(f"{side}_gripper", finger_joints(side), [want, want], 0.5, first=first)
        self._pinch = {**getattr(self, "_pinch", {}), side: want}
        return ok

    def finger_position(self, side):
        """Half the gap between the pads once the fingers have stopped:
        the mean of the two joints, because each finger is its own spring
        and a bar 6 mm off the hand's centre line stops one finger at
        +0.007 and lets the other through to -0.001 (measured, and read
        from that one finger alone it passed for a miss)."""
        self._spin(0.3)
        pos = [self._joint(j) for j in finger_joints(side)]
        if None in pos:
            return None
        pos = sum(pos) / 2.0
        self.log.info(f"{side} fingers closed to {pos:.4f}")
        return pos

    def closed_on_something(self, side, minimum=FINGER_JAR_MIN):
        pos = self.finger_position(side)
        return pos is not None and pos > minimum

    def carry(self, side):
        """The hand, holding something, to CARRY_TCP with the forward grip,
        level -- or, when that has no plan, the tuck (a tilted jar is
        better than a jar on the floor). True when it is there."""
        x, y, z = CARRY_TCP
        goal = (x, y if side == "left" else -y, z)
        here = self.tcp_position(side)
        if here is not None and math.dist(here, goal) < 0.03 and self.holding[side]:
            return True
        if not self.lift_to(0.0):
            return False
        if self.move_tcp(side, goal, GRASP_FORWARD, tol_orient=(0.1, 0.1, 0.1), label="carry"):
            return True
        self.log.warn(f"{side} arm: no plan to the carry pose; tucking instead")
        return self.posture(side, "tuck")

    def posture(self, side, name):
        angles = k.POSTURES[name][side]
        m = self.moveit[side]
        m.max_velocity = m.max_acceleration = HOLD_SPEED if self.holding[side] else FREE_SPEED
        # already there (every goto tucks both arms, and usually they are)
        now = [self._joint(j) for j in arm_joints(side)]
        if None not in now and all(abs(a - angles[j]) < 0.03
                                   for a, j in zip(now, k.ARM_JOINTS)):
            return True
        # two goes: RRTConnect's path now and then fails MoveIt's own
        # validation one state in fifty (INVALID_MOTION_PLAN, the camera
        # link clipping a worktop slab) and the next plan is fine
        for _ in range(2):
            m.move_to_configuration([angles[j] for j in k.ARM_JOINTS])
            if m.wait_until_executed():
                return True
        return False

    def move_tcp(self, side, position, quat=GRASP_FORWARD, cartesian=False,
                 tol_orient=ORIENT_TOL, label="", slow=False, fallback=True, speed=None,
                 tol_fallback=0.15, record=None):
        """Plan and execute a TCP goal in base_link; True when executed.
        A planned move runs at FREE_SPEED, a straight one at CART_SPEED,
        slow=True at SLOW_SPEED (see the constants).

        Humble's Cartesian path service ignores the scaling factors (the
        request has none until Iron) and times the path at the joints'
        full limits, so a straight path is retimed here to `speed`: until
        it was, every "slow" pull on the door and every lift of a jar ran
        flat out (measured: the 7 deg pull step in 0.4 s).

        A Cartesian path whose IK flips configuration between two
        waypoints is rejected (jump threshold): the controller executes
        such a jump at full speed, and that has slammed the wrist into the
        door and, once, spun the shoulder one and a half turns retreating
        from the microwave (measured: joint1 9.7 rad out). A rejected
        straight line is planned instead, with the orientation held to 0.15
        rad, unless fallback=False (a hand on the door must move straight
        or not at all). `record`, a list, gets the executed trajectory
        (see retrace)."""
        m = self.moveit[side]
        m.cartesian_jump_threshold = 3.0
        self.log.info(f"{side} arm -> ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                      f"{' cartesian' if cartesian else ''} {label}")
        if cartesian and quat is None:
            quat = self.tcp_quat(side) or GRASP_FORWARD
        # a jar in hand pivots in the pads (2.5 cm tall on an 11.5 cm jar)
        # and a free swing at 0.4 flung one across the kitchen (measured);
        # holding something, a planned swing goes at HOLD_SPEED
        if speed is None:
            speed = (SLOW_SPEED if slow else CART_SPEED if cartesian
                     else HOLD_SPEED if self.holding[side] else FREE_SPEED)
        m.max_velocity = m.max_acceleration = speed
        pos = tuple(float(v) for v in position)
        traj = m.plan(position=pos, quat_xyzw=quat, tolerance_position=0.008,
                      tolerance_orientation=tol_orient, cartesian=cartesian, max_step=0.01,
                      cartesian_fraction_threshold=0.9 if cartesian else 0.0)
        if traj is None and cartesian and fallback:
            # the orientation held to `tol_fallback`: 0.15 rad as a rule,
            # 0.05 for the pre-place (a jar that arrives at the microwave's
            # mouth 8 deg off level is levelled by the straight move in,
            # and its rim swept the mouth's edge on the way -- measured:
            # the jar knocked over, 18 cm off its spot; at 0.05 everywhere
            # a retreat from a jar grasp found no plan in 12 s, measured)
            self.log.info(f"{side} arm: no straight path for '{label or 'move'}'; "
                          "planning one instead")
            traj = m.plan(position=pos, quat_xyzw=quat, tolerance_position=0.008,
                          tolerance_orientation=tol_fallback, cartesian=False)
        if traj is None:
            self.log.warn(f"{side} arm: {label or 'move'}: no plan")
            return False
        if cartesian:
            retime(traj, 1.0 / speed)
        m.execute(traj)
        ok = m.wait_until_executed()
        if not ok:
            self.log.warn(f"{side} arm: {label or 'move'} failed")
            self.relax(side)
        elif record is not None:
            record.append(traj)
        return ok

    def retrace(self, side, way, label="retrace"):
        """Run recorded moves (move_tcp's `record`) backwards, last first:
        the hand leaves by exactly the joint path it came in on, which was
        collision-free with a jar in it and is so without. Nothing is
        planned, so nothing new can go wrong in a tight spot. True when all
        of it ran."""
        m = self.moveit[side]
        for traj in reversed(way):
            here = [self._joint(j) for j in traj.joint_names]
            back = reversed_path(traj, None if None in here else here)
            took = back.points[-1].time_from_start
            self.log.info(f"{side} arm: {label} ({len(back.points)} points, "
                          f"{took.sec + took.nanosec * 1e-9:.1f} s)")
            m.execute(back)
            if not m.wait_until_executed():
                self.log.warn(f"{side} arm: {label} failed")
                self.relax(side)
                return False
        return True

    def cartesian_path(self, side, points, quat, speed, label=""):
        """One straight-segment path through several TCP points (base_link),
        the orientation held at `quat`, executed at `speed` (retimed as
        move_tcp's are). Returns how much of it was executed (the planned
        fraction, or 0.0 when there was no plan or the execution failed).

        pymoveit2 sends a single waypoint; MoveIt's service takes several,
        and a pull made of five 2 cm moves pays five accelerations and five
        stops -- 1.9 s each, where the whole arc in one takes about half."""
        from geometry_msgs.msg import Pose
        from moveit_msgs.srv import GetCartesianPath
        if not hasattr(self, "_cart_client"):
            self._cart_client = self.node.create_client(GetCartesianPath,
                                                        "/compute_cartesian_path")
        if not self._cart_client.wait_for_service(timeout_sec=2.0):
            return 0.0
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.start_state.is_diff = True
        req.group_name = f"{side}_arm"
        req.link_name = f"openarmx_{side}_hand_tcp"
        for pt in points:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in pt)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
            req.waypoints.append(pose)
        req.max_step = 0.01
        req.jump_threshold = 3.0
        req.avoid_collisions = True
        self.log.info(f"{side} arm -> {len(points)} points, cartesian {label}")
        fut = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or res.error_code.val != 1 or res.fraction <= 0.0:
            return 0.0
        traj = res.solution.joint_trajectory
        retime(traj, 1.0 / speed)
        m = self.moveit[side]
        m.execute(traj)
        if not m.wait_until_executed():
            self.log.warn(f"{side} arm: {label} failed")
            self.relax(side)
            return 0.0
        return float(res.fraction)

    def relax(self, side):
        """Re-command the arm to where it actually is. A trajectory the
        controller aborted leaves it holding the LAST commanded point, which
        for an arm stopped by a counter means 120 N.m still pushing on it --
        enough to tip the whole robot (it did, once)."""
        js = self.moveit[side].joint_state
        if js is None:
            return
        pos = dict(zip(js.name, js.position))
        names = arm_joints(side)
        client = ActionClient(self.node, FollowJointTrajectory,
                              f"/{side}_arm_controller/follow_joint_trajectory")
        if client.wait_for_server(timeout_sec=2.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(pos[j]) for j in names]
            pt.time_from_start = Duration(sec=0, nanosec=300_000_000)
            goal.trajectory.points = [pt]
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=2.0)
            self._spin(0.4)
        client.destroy()

    def free(self, side, posture="tuck"):
        """Last resort for an arm that MoveIt cannot move (a hand jammed
        under a door edge, once): relax, open the hand, then walk the joints
        to `posture` one at a time from the wrist inwards, straight through
        the controller. No collision checking -- the wrist moves first so
        the hand backs out of whatever it is in before the big joints swing."""
        self.log.warn(f"{side} arm: freeing it joint by joint")
        self.relax(side)
        self.gripper(side, k.FINGER_OPEN)
        # column up first: a wrist that will not turn is usually a hand
        # against the chassis (measured: joint7 stuck at -0.7 with the
        # column 0.27 m down, free with it up)
        self.lift_to(0.0)
        js = self.moveit[side].joint_state
        if js is None:
            return False
        pos = dict(zip(js.name, js.position))
        names = arm_joints(side)
        target = [k.POSTURES[posture][side][j] for j in k.ARM_JOINTS]
        current = [float(pos[j]) for j in names]
        client = ActionClient(self.node, FollowJointTrajectory,
                              f"/{side}_arm_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=2.0):
            return False
        for i in reversed(range(len(names))):
            current[i] = target[i]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = list(current)
            pt.time_from_start = Duration(sec=2, nanosec=500_000_000)
            goal.trajectory.points = [pt]
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=2.0)
            handle = send.result()
            if handle is not None and handle.accepted:
                result = handle.get_result_async()
                rclpy.spin_until_future_complete(self.node, result, timeout_sec=10.0)
            js = self.moveit[side].joint_state
            pos = dict(zip(js.name, js.position))
            current = [float(pos[j]) for j in names]
        client.destroy()
        err = max(abs(c - t) for c, t in zip(current, target))
        self.log.info(f"{side} arm: freed to within {err:.2f} rad of {posture}")
        return err < 0.1

    # ------------------------------------------------------------ planning scene
    def _scene_has(self, object_id, attached=False, present=True, timeout=3.0):
        """Poll move_group until the object is (or is not) in the scene."""
        m = self.moveit["left"]
        end = time.time() + timeout
        while time.time() < end:
            if m.update_planning_scene():
                scene = m.planning_scene
                if attached:
                    ids = [o.object.id for o in scene.robot_state.attached_collision_objects]
                else:
                    ids = [o.id for o in scene.world.collision_objects]
                if (object_id in ids) == present:
                    return True
            self._spin(0.1)
        self.log.warn(f"planning scene never showed {object_id} "
                      f"{'attached' if attached else ''}{'' if present else ' removed'}")
        return False

    def add_box(self, object_id, size, center, quat=(0.0, 0.0, 0.0, 1.0)):
        self.moveit["left"].add_collision_box(id=object_id, size=tuple(size),
                                              position=tuple(center),
                                              quat_xyzw=quat, frame_id="base_link")
        return self._scene_has(object_id)

    def remove_object(self, object_id):
        self.moveit["left"].remove_collision_object(object_id)
        return self._scene_has(object_id, present=False)

    def attach(self, side, object_id):
        self.moveit[side].attach_collision_object(
            id=object_id, link_name=f"openarmx_{side}_hand",
            touch_links=[f"openarmx_{side}_hand", f"openarmx_{side}_left_finger",
                         f"openarmx_{side}_right_finger", f"{side}_wrist_camera_link"])
        ok = self._scene_has(object_id, attached=True)
        self.holding[side] = object_id
        return ok

    def detach(self, side):
        object_id = self.holding[side]
        if object_id is None:
            return True
        self.moveit[side].detach_collision_object(object_id)
        ok = self._scene_has(object_id, attached=True, present=False)
        self.holding[side] = None
        return ok

    # ------------------------------------------------------------ skills
    # ------------------------------------------------------------ arm + column
    def _with_lift(self, side):
        """The side's arm AND the column as one planning group (SRDF
        *_arm_lift): a plan that moves both at once, split by move_group
        between the lift and the arm controllers."""
        if not hasattr(self, "_lift_groups"):
            self._lift_groups = {}
        if side not in self._lift_groups:
            m = MoveIt2(node=self.node, joint_names=["lift_joint"] + arm_joints(side),
                        base_link_name="base_link", end_effector_name=f"openarmx_{side}_hand_tcp",
                        group_name=f"{side}_arm_lift")
            m.allowed_planning_time = 6.0
            m.num_planning_attempts = 5
            self._lift_groups[side] = m
        return self._lift_groups[side]

    def ik_at_lift(self, side, position, quat, lift, seed="ready"):
        """The arm's seven joints that put the TCP at `position`/`quat`
        (base_link) with the column at `lift`, collision-free in the current
        scene, or None. Seeded from a named posture, so the answer is the
        arm reaching forward rather than whatever odd elbow is nearest the
        tuck."""
        js = self.moveit[side].joint_state
        if js is None:
            return None
        from sensor_msgs.msg import JointState
        state = JointState()
        state.name = list(js.name)
        state.position = list(js.position)
        seed_angles = k.POSTURES[seed][side] if seed else None
        for i, name in enumerate(state.name):
            if name == "lift_joint":
                state.position[i] = float(lift)
            for j in k.ARM_JOINTS:
                if seed_angles and name == f"openarmx_{side}_{j}":
                    state.position[i] = float(seed_angles[j])
        sol = self.moveit[side].compute_ik(tuple(float(v) for v in position),
                                           tuple(float(v) for v in quat),
                                           start_joint_state=state)
        if sol is None:
            return None
        names = list(sol.name)
        try:
            return [sol.position[names.index(j)] for j in arm_joints(side)]
        except ValueError:
            return None

    def move_with_lift(self, side, lift, arm, label="", speed=None):
        """Arm and column together to a joint configuration: one motion
        instead of a column move, an arm move and another column move."""
        now = [self._joint("lift_joint")] + [self._joint(j) for j in arm_joints(side)]
        if None not in now and abs(now[0] - lift) < 0.005 and all(
                abs(a - b) < 0.03 for a, b in zip(now[1:], arm)):
            return True                  # already there (every goto tucks, and usually is)
        m = self._with_lift(side)
        if speed is None:
            speed = HOLD_SPEED if self.holding[side] else FREE_SPEED
        m.max_velocity = m.max_acceleration = speed
        self.log.info(f"{side} arm + column -> lift {lift:+.3f} {label}")
        for _ in range(2):
            m.move_to_configuration([float(lift)] + [float(a) for a in arm])
            if m.wait_until_executed():
                return True
        self.log.warn(f"{side} arm + column: {label} failed")
        return False

    def reach(self, side, position, quat, lift, label="", rise=0.0, column_first=False):
        """The hand to a pose with the column at `lift`, arm and column
        together. False when there is no IK or no plan (the caller then
        falls back to reach_out, which moves them one at a time).

        rise > 0: the arm takes the pose it will have at `position`, but the
        column stops `rise` higher, so the hand arrives that far ABOVE
        `position`; lowering the column then brings it straight down onto it.
        The arm's IK is the target's own -- a hover point that is a pose of
        its own had none 8 cm over the pre-grasp (measured).

        column_first: the column goes to its height before the arm moves,
        with the arm still where it is -- for a reach whose column comes
        DOWN while the arm swings out over a worktop: moving together, the
        arm lagged its plan while the column dropped, and the wrist caught
        the counter's edge (measured, one pick in ten).

        IK is seeded from the reaching posture, then from wherever the arm
        is, then from the tuck: KDL only searches near its seed, and the
        fist under the open door had no answer from the first two that the
        planner then found from the tuck (measured, every close)."""
        arm = None
        for seed in ("ready", None, "tuck"):
            arm = self.ik_at_lift(side, position, quat, lift, seed=seed)
            if arm is not None:
                break
        if arm is None:
            self.log.warn(f"{side} arm: no IK for {label} with the column at {lift:+.3f}")
            return False
        if column_first and not self.lift_to(lift + rise):
            return False
        return self.move_with_lift(side, lift + rise, arm, label)

    def bring_back(self, side, lift=0.0):
        """Arm and column home together: the tuck for an empty hand, the
        level carry pose (CARRY_TCP) for a hand holding something."""
        if self.holding[side]:
            x, y, z = CARRY_TCP
            goal = (x, y if side == "left" else -y, z)
            # already there, by the hand's position (the arm is redundant: a
            # fresh IK answer is a slightly different elbow, and comparing
            # joints re-planned a 1 s move to where the hand already was,
            # three times a goto)
            here, now = self.tcp_position(side), self._joint("lift_joint")
            if here is not None and now is not None and math.dist(here, goal) < 0.015 \
                    and abs(now - lift) < 0.005:
                return True
            arm = self.ik_at_lift(side, goal, GRASP_FORWARD, lift, seed="tuck")
            if arm is None:
                return self.carry(side)
            # the column first when it has far to rise (after a pick from low
            # down): together, the arm dipped under its plan on the way back
            # over the worktop, the jar's bottom scraped along it, and the jar
            # slid 3 cm out along the pads (measured)
            if now is not None and lift - now > 0.05 and not self.lift_to(lift):
                return self.carry(side)
            if not self.move_with_lift(side, lift, arm, "carry"):
                return self.carry(side)
            return True
        arm = [k.POSTURES["tuck"][side][j] for j in k.ARM_JOINTS]
        if self.move_with_lift(side, lift, arm, "tuck"):
            return True
        return self.posture(side, "tuck") and self.lift_to(lift)

    def reach_out(self, side, pre, lift_height, quat=GRASP_FORWARD, up=0.0):
        """Get the hand to `pre` (base_link, where the base will have the
        column at `lift_height`) without ever passing below the worktop.

        The column is raised first: with it up, the tucked hand and the
        hover point above `pre` are both above worktop height, so the plan
        from one to the other is easy (with the column down, the tuck sits
        below the worktop behind the counter and every plan out had to
        thread the edge -- measured, one success in three). Then the column
        comes down with the arm already out, which lowers the hand in
        base_link by the same amount, and the last VIA_ABOVE is a straight
        descent."""
        if not self.lift_to(up):
            return False
        # where the hover must be while the column is up, so that it sits at
        # pre + VIA_ABOVE once the column is at lift_height -- but never
        # above the shoulder: the forward grip has no IK there (measured:
        # a place 8 cm higher than a pick put the hover 5 cm over the
        # shoulder and every plan failed), and a hover that ends a few
        # centimetres nearer `pre` costs nothing
        # (0.10 below: at 0.03 below the hover had IK from 0.62 m ahead and
        # none from 0.58 m, and 4 cm is one relocalization)
        shoulder_z = k.shoulder_in_base(side, up)[2]
        # Near enough straight above `pre` (4 cm ahead, the descent slants
        # that much) so the column's descent is over the spot the hand is
        # going to -- but never nearer the body than HOVER_MIN_X, where the
        # elbow cannot fold to it (no plan at 0.56 and 0.60 m ahead, one at
        # 0.64, measured, 12 s a failure). Then 4 and 8 cm further out.
        hover_z = min(pre[2] + VIA_ABOVE + (up - lift_height), shoulder_z - 0.10)
        first = max(0.04, HOVER_MIN_X - pre[0])
        for ahead in (first, first + 0.04, first + 0.08):
            if self.move_tcp(side, (pre[0] + ahead, pre[1], hover_z), quat,
                             label="hover, column up"):
                break
        else:
            return False
        if not self.lift_to(lift_height):
            return False
        # the exact grip from here on: the planner's 0.25 rad of pitch on the
        # hover, kept through the straight moves, once put the wrist 15 cm
        # behind the hand 4 cm lower -- onto the open door's edge, from
        # where the retreat blew the arm up (measured: contact at the edge)
        return self.move_tcp(side, pre, quat, cartesian=True, label="pre-grasp",
                             tol_fallback=0.05)

    def pick(self, side, target, lift_height, approach=(-0.15, 0.0, 0.0),
             radius=0.045, height=0.115):
        """Side grasp of a jar whose axis passes through `target` (base_link,
        at the jar's mid-height, as seen with the column at `lift_height`).
        The pads take it GRIP_ABOVE_MID higher. Returns OK, MISSED (fingers
        met nothing) or FAIL."""
        mid = target
        target = (mid[0], mid[1], mid[2] + GRIP_ABOVE_MID)
        pre = tuple(t + a for t, a in zip(target, approach))
        jar = f"jar_{side}"
        # the column on its way down to the reach's height already, while the
        # scene and the IK are set up (reach's column_first waits for it)
        self._send("lift", ["lift_joint"], [lift_height + PICK_HOVER],
                   1.0 + abs(lift_height + PICK_HOVER - (self._joint("lift_joint") or 0.0)) / 0.08)
        self.remove_object(jar)
        # One motion, arm and column together, from the tuck to 15 cm in front
        # of the jar -- with the jar itself in the scene meanwhile, so that a
        # reach no longer made through a hover above it cannot pass through
        # it. The old column-up, hover, column-down reach is the fallback.
        # The fingers are shut for the swing and open while the column comes
        # down: open, the worktop's edge got between them twice in ten
        # errands, pried them 2 cm past open and held the hand, and every
        # arm move after that aborted until it was walked out joint by joint
        # (measured, twice in ten errands).
        self.open_while_moving(side, k.FINGER_CLOSED)
        self.add_box(f"{jar}_target", (2 * radius, 2 * radius, height), mid)
        # to PICK_HOVER above `pre` in the one motion, then the column straight
        # down onto it: the pre-grasp is 6 cm over the worktop, a planned
        # swing dips a few centimetres under its plan, and the wrist struck
        # the worktop's edge on the way in (joint 5 half a radian off its
        # path, the reach aborted and the column-up fallback took 11 s more;
        # measured). The column's travel does not dip.
        reached = self.reach(side, pre, GRASP_FORWARD, lift_height, "above the pre-grasp",
                             rise=PICK_HOVER, column_first=True)
        if reached:
            self.open_while_moving(side, k.FINGER_OPEN)
            reached = self.lift_to(lift_height)
        self.remove_object(f"{jar}_target")
        if not reached and not self.reach_out(side, pre, lift_height):
            return FAIL
        if not self.gripper(side, k.FINGER_OPEN):
            return FAIL
        if not self.move_tcp(side, target, GRASP_FORWARD, cartesian=True, label="grasp"):
            return FAIL
        self.grasp(side)
        if not self.closed_on_something(side):
            self.log.warn(f"{side} hand closed on nothing")
            self.gripper(side, k.FINGER_OPEN)
            self.move_tcp(side, pre, None, cartesian=True, label="retreat")
            return MISSED
        # the jar joins the robot: a box in the hand, so plans keep it clear
        self.add_box(jar, (2 * radius, 2 * radius, height), mid)
        self.attach(side, jar)
        # From here the pick has happened whatever the arm manages next:
        # a lift or a retreat that fails leaves the jar in the hand, and
        # the caller's carry pose is planned from wherever that is (a pick
        # that reported failure with the jar in hand had the retry try to
        # pick it again, measured). Only an empty hand is a miss.
        # Up off the worktop and back out along the way in, both straight;
        # the caller then brings the arm and the column home together
        # (bring_back), which replaces the old straight climb and separate
        # column move.
        lifted = tuple(t + a for t, a in zip(target, (0.0, 0.0, 0.06)))
        back = [t + a for t, a in zip(lifted, approach)]
        # never nearer the body than the elbow folds to (HOVER_MIN_X): the
        # last 2 cm of a 15 cm retreat from a jar 0.77 m out had no IK, and
        # the planned fallback cost 4 s (measured)
        back[0] = max(back[0], min(lifted[0], HOVER_MIN_X + 0.005))
        back = tuple(back)
        if self.move_tcp(side, lifted, None, cartesian=True, label="lift"):
            self.move_tcp(side, back, None, cartesian=True, label="retreat")
        if not self.closed_on_something(side):
            self.log.warn(f"{side} hand lost the jar")
            self.detach(side)
            self.remove_object(jar)
            return MISSED
        return OK

    def place(self, side, target, lift_height, approach=(-0.15, 0.0, 0.0), above=0.05,
              retreat="arm", up=0.0, lead_in=None):
        """Put what the hand holds down with its axis at `target` (base_link,
        the jar's mid-height when down; the pads hold it GRIP_ABOVE_MID
        higher, see pick). How the open hand then leaves: "arm", a straight
        move back to the pre-place point; "retrace", the two straight moves
        that brought it in run backwards (retrace) -- how a hand leaves a
        microwave; "base", not at all: the caller backs the base out."""
        target = (target[0], target[1], target[2] + GRIP_ABOVE_MID)
        over = tuple(t + a for t, a in zip(target, (0.0, 0.0, above)))
        pre = tuple(t + a for t, a in zip(over, approach))
        # lead_in (dx, dz), the microwave: in one planned motion to `pre` +
        # (dx, 0, dz) -- the arm posed for `pre` + (dx, 0, 0), the column dz
        # higher (reach's rise) -- straight forwards to above `pre` (unless dx
        # is 0), and the column straight down onto it. The flat open door
        # lies 2 cm under the way in; the one-motion
        # reach straight to `pre`, planned to clear it by that much, dipped
        # under its plan with the jar in hand, caught the door's tip from
        # below and lifted it from 86 to 15 deg, and the jar slid 3 cm in the
        # pads (twice in four runs, measured). Up and over by straight lines,
        # the jar never goes near the tip. And not column-first (reach_out):
        # its hover, which the elbow cannot bring nearer than 0.64 m, has the
        # jar's front 3 cm under the microwave's roof line once the column is
        # down (measured). reach_out stays the fallback.
        if lead_in is not None:
            lx, lz = lead_in
            high = (pre[0], pre[1], pre[2] + lz)
            done = (self.reach(side, (pre[0] + lx, pre[1], pre[2]), GRASP_FORWARD,
                               lift_height, "above the way in", rise=lz)
                    and (abs(lx) < 1e-3
                         or self.move_tcp(side, high, GRASP_FORWARD, cartesian=True,
                                          label="over the door", fallback=False))
                    and self.lift_to(lift_height))
            if not done and not self.reach_out(side, pre, lift_height, up=up):
                return FAIL
        elif (not self.reach(side, pre, GRASP_FORWARD, lift_height, "pre-place")
                and not self.reach_out(side, pre, lift_height, up=up)):
            return FAIL
        way_in = []
        if not self.move_tcp(side, over, GRASP_FORWARD, cartesian=True, label="over the spot",
                             record=way_in):
            return FAIL
        if not self.move_tcp(side, target, GRASP_FORWARD, cartesian=True, label="set down",
                             record=way_in):
            return FAIL
        if self.probe:
            self.probe("set down")
        self.gripper(side, k.FINGER_OPEN)
        self.detach(side)
        self.remove_object(f"jar_{side}")
        if self.probe:
            self.probe("released")
        # back out the way it came in, at the height it came in at: a
        # diagonal retreat from inside the microwave lifted the wrist
        # camera into the cavity's ceiling (measured)
        # -- and once the jar is down the place has succeeded whatever the
        # retreat does: the caller tucks the arm with the planner, which
        # knows the scene; a retry would try to put down a jar it no longer
        # holds
        if retreat == "base":
            return OK
        # A hand in a microwave leaves the way it came in, joint for joint.
        # A fresh straight path out of that stretched, cavity-bounded pose
        # failed one time in two, and the planned alternative swung the arm
        # into the cavity wall (measured); the base backing out instead
        # worked, but left it 0.4 m from where the door is closed from.
        if retreat == "retrace":
            self.retrace(side, way_in, "out the way it went in")
            return OK
        back = (pre[0], pre[1], over[2])
        if self.move_tcp(side, back, GRASP_FORWARD, cartesian=True, label="retreat",
                         fallback=False):
            self.move_tcp(side, pre, None, cartesian=True, label="up")
        # (the caller brings the arm and the column home together)
        return OK

    def door(self, side, handle, hinge, opening, lift_height, angle=0.6, steps=5,
             opened=None):
        """Grip the handle bar and swing it round the hinge (both in base_link;
        the hinge axis is along y). opening=True swings from closed to
        `angle` and lets go: past 10 deg the door's own weight carries it
        the rest of the way to its stop (1.5 rad, lying flat), and the
        fixed-orientation wrist runs out of range near 45 deg anyway
        (measured: the 48 deg step had no IK from the working standoff).
        Use door_close() for the other direction -- the bar is not
        grippable under a flat door. `opened()`, when given, is asked
        after a grip that missed: a hand that closed beside the bar
        rather than on it hooks it on the way back, and that opens the
        door just as well (measured, four runs in four)."""
        lift_bar = [0.0]          # the measured bar's height over the fixture's

        def handle_at(a):
            h = (handle[0], handle[1], handle[2] + lift_bar[0])
            return door_handle_at(h, hinge, a)

        def roll_at(a):
            return quat_mul(quat_about_y(-a), DOOR_GRIP)

        def grip_at(a, short):
            return tuple(p + d for p, d in zip(handle_at(a), (-short, 0.0, 0.0)))

        start = 0.0 if opening else angle

        # The fingers open while arm and column travel together to `pre`
        # (reach); if that has no IK or no plan, the old way: the column to
        # height first, then the planner takes the arm to pre. (No hover
        # above `pre`: with the column where the grip has IK -- see
        # DOOR_SHOULDER_ABOVE -- a point 12 cm higher has none.)
        self.open_while_moving(side, DOOR_FINGER_OPEN)
        at_height = False
        # Two rolls of the same grip (pads across the bar either way round,
        # the wrist camera to the left or to the right): a free roll let the
        # planner pick one from which the straight approach had no IK, so
        # an approach that fails switches roll. A closing that jams on the
        # door's face comes back and goes in a centimetre shorter. Four
        # goes: a failed step costs the mission forty seconds, a failed go
        # here costs five.
        rolls = [roll_at(start), quat_mul(roll_at(start), (0.0, 0.0, 1.0, 0.0))]
        short, which, gripped, measured = DOOR_GRIP_SHORT, 0, False, False
        for _ in range(4):
            roll = rolls[which]
            pre = tuple(p + d for p, d in zip(grip_at(start, short),
                                              (-DOOR_PRE_BACK, 0.0, 0.0)))
            # with the exact roll, straight or not at all: the planned
            # alternative once ended with the fingers on the door's face
            if not at_height:
                at_height = True
                arrived = (self.reach(side, pre, roll, lift_height, "before the handle")
                           or (self.lift_to(lift_height)
                               and self.move_tcp(side, pre, roll, tol_orient=DOOR_PRE_TOL,
                                                 label="before the handle")))
            else:
                arrived = self.move_tcp(side, pre, roll, tol_orient=DOOR_PRE_TOL,
                                        label="before the handle")
            if not arrived:
                which = 1 - which
                continue
            self.gripper(side, DOOR_FINGER_OPEN)
            # Standing at `pre` the wrist camera is looking straight at the
            # door from 20 cm, which is the one place the panel's real
            # distance can be had. Once only: after a jam the hand has
            # already touched the door and `short` is being stepped back
            # deliberately.
            if not measured:
                measured = True
                seen = self.measure_door_short(side, handle_at(start))
                if seen is not None:
                    short, lift_bar[0] = seen
            want = grip_at(start, short)
            if not self.move_tcp(side, want, roll, cartesian=True,
                                 label="on the handle", fallback=False):
                which = 1 - which
                continue
            here = self.tcp_position(side)
            if here is not None:
                self.log.info(
                    f"door: TCP wanted {want[0]:.4f} {want[1]:.4f} {want[2]:.4f}, "
                    f"got {here[0]:.4f} {here[1]:.4f} {here[2]:.4f} "
                    f"(along the approach {here[0] - want[0]:+.4f})")
            self.grasp(side, k.FINGER_SQUEEZE_HARD)
            pos = self.finger_position(side)
            if pos is not None and FINGER_BAR_MIN < pos < FINGER_BAR_MAX:
                gripped = True
                break
            missed = pos is not None and pos <= FINGER_BAR_MIN
            self.log.warn(f"{side} hand "
                          f"{'missed the handle' if missed else 'jammed on the door face'}")
            self.gripper(side, DOOR_FINGER_OPEN)
            self.move_tcp(side, pre, roll, cartesian=True, label="retreat")
            self._spin(1.0)
            if opened and opened():
                self.log.info(f"{side} hand hooked the door open on the way back")
                return OK
            # a jam is the tips on the panel: shallower. A miss is the pads
            # short of the bar or beside it: deeper, but never so deep that
            # the tips would reach the panel.
            if missed:
                short = max(DOOR_SHORT_RANGE[0], short - 0.004)
            else:
                short += 0.006
        if not gripped:
            return MISSED
        # The hand keeps its orientation along the arc: the 8 mm bar pivots
        # between the flat pads, and a wrist that tried to turn with the door
        # ran out of range at 37 degrees (measured).
        # Past 10 deg the door's own weight finishes the opening, so a step
        # that fails after that (the wrist's IK band ends somewhere between
        # 20 and 45 deg, depending on the base's exact spot) is a job done,
        # not a failure: let go and let it fall. The caller checks.
        end = 0.0 if opening else angle
        first = 1
        if opening:
            # the whole arc in one path; whatever it does not reach, and a
            # path that failed on the way, goes on step by step from the
            # nearest step to where the hand is
            pts = [grip_at(angle * i / steps, short) for i in range(1, steps + 1)]
            done = self.cartesian_path(side, pts, self.tcp_quat(side) or roll,
                                       DOOR_PULL_SPEED, "the door's arc")
            here = self.tcp_position(side)
            if done > 0.0 or here is None:
                reached = int(done * steps + 1e-6)
            else:
                reached = min(range(steps + 1), key=lambda i: math.dist(
                    here, grip_at(angle * i / steps, short) if i else want))
            if reached:
                end = angle * reached / steps
                first = reached + 1
        for i in range(first, steps + 1):
            a = angle * i / steps if opening else angle * (1 - i / steps)
            # the first contact is already made and the pads are round the
            # bar, so the care that SLOW_SPEED bought is spent; DOOR_PULL_SPEED
            # is still well under the speed that flicked the door 40 deg into
            # the air on first touch
            if not self.move_tcp(side, grip_at(a, short), None, cartesian=True,
                                 tol_orient=0.4, label=f"door {math.degrees(a):.0f} deg",
                                 speed=DOOR_PULL_SPEED, fallback=False):
                if not (opening and end > 0.2):
                    self.gripper(side, k.FINGER_OPEN)
                    return FAIL
                self.log.info(f"{side} arm: letting the door fall from "
                              f"{math.degrees(end):.0f} deg")
                break
            end = a
        # Let go, and leave the leaving to the BASE (see the mission's door
        # verb, which backs it out DOOR_RELEASE_BACK). Opened, the fingers let
        # the door down onto the lower one, where it settles at about 55 deg
        # with its top edge on the finger. Any move that raises that finger
        # lifts the door: the up-and-back that suited the old grip (the bar
        # on the pads' last millimetre, off them at once) has to lift this
        # one 16 mm before the bar clears the pad, and it pushed the door back
        # to 34 deg every run, rocked the robot 2.4 deg in pitch on its
        # lighter base -- the carried jar swinging 3 cm -- and held the door
        # up on the finger until the tuck swept the arm through where it was
        # about to fall (measured; once it cost the jar). A finger that slides
        # out BACKWARDS from under the door never pushes on it: the door
        # follows it down and settles on its stop.
        self.gripper(side, k.FINGER_OPEN)
        return OK

    def door_close(self, side, handle, hinge, lift_height, start=DOOR_OPEN,
                   end=DOOR_CLOSED_ENOUGH, steps=10, on_step=None, shut=None):
        """Shut a dropped door: the closed fist lifts the door by its face
        round the hinge (base_link coordinates, closed-door handle and
        hinge as for door()). Pushes on to `end`, past closed, so the door
        is pressed home -- unless `shut()` says it already is: past 45 deg
        the door falls the rest of the way on its own (measured: shut from
        the 65 deg step), and the steps left would press on a closed door
        for fifteen seconds."""
        def under(a):
            return tuple(p + d for p, d in zip(door_point(hinge, DOOR_PUSH_CORNER, a),
                                               DOOR_PUSH_OFFSET))

        # the fist closes while arm and column travel together to the first
        # `pre` (as for door(): no column-up hover, nothing under it but air)
        self._send(f"{side}_gripper", finger_joints(side),
                   [k.FINGER_CLOSED, k.FINGER_CLOSED], 1.2)
        at_height = False
        # The way in, under the bar: at each dip in turn, the planner
        # takes the fist to `pre` (from wherever a stopped approach left
        # it) and the exact pitch from there on -- the planner's tolerance
        # on the approach would tilt the flat. An approach the controller
        # stops (the flat met the bar or the door's edge) is tried again
        # a centimetre off, not given up on.
        for dip in DOOR_PUSH_DIPS:
            low = tuple(p + d for p, d in zip(under(start), (0.0, 0.0, -dip)))
            pre = tuple(p + d for p, d in zip(low, (-0.12, 0.0, 0.0)))
            if not at_height:
                at_height = True
                arrived = (self.reach(side, pre, FIST_UP, lift_height, "before the door")
                           or (self.lift_to(lift_height)
                               and self.move_tcp(side, pre, FIST_UP,
                                                 label="before the door, fist up")))
            else:
                arrived = self.move_tcp(side, pre, FIST_UP, label="before the door, fist up")
            if not arrived:
                return FAIL
            self.gripper(side, k.FINGER_CLOSED)
            if self.move_tcp(side, low, FIST_UP, cartesian=True, label="under the door's edge",
                             fallback=False):
                break
            self.log.info(f"{side} arm: something stopped the fist {dip * 100:.0f} cm under "
                          "the door; trying another height")
        else:
            return FAIL
        if not self.move_tcp(side, under(start), FIST_UP, cartesian=True,
                             label="up against the face",
                             slow=True):
            return FAIL
        # The arc rises 10 cm and the grip's IK band below the shoulder is
        # not much wider: the column climbs with the hand (60% of the
        # rise, the arm keeps 40%), which also lifts the door. The last
        # steps press the door onto its frame and the
        # controller aborts a few degrees short; past 60% of the arc that
        # is what closing feels like, not a failure. The caller checks.
        done = 0
        z0 = under(start)[2]
        lift_now = lift_height
        missed = 0
        pushes = []
        for i in range(1, steps + 1):
            a = start + (end - start) * i / steps
            # the column only moves when it is worth a trajectory: a lift_to
            # costs a second of overhead however small the move, and fifteen
            # of them, serialised with the arm, were a fifth of the step
            want = lift_height + 0.6 * (under(a)[2] - z0)
            if abs(want - lift_now) > 0.03:
                self.lift_to(want)
                lift_now = want
            pushed = self.move_tcp(side, under(a), None, cartesian=True, tol_orient=0.4,
                                   label=f"door up to {math.degrees(a):.0f} deg",
                                   speed=DOOR_PUSH_SPEED, fallback=False, record=pushes)
            if not pushed and abs(want - lift_now) > 0.005:
                # the column lags its share by up to 3 cm between moves, and
                # at the edge of the band the step to 45 deg ran out of IK
                # part way, three times in five errands, and the door fell
                # back open (measured); the column exactly where it should
                # be, the same step again
                self.lift_to(want)
                lift_now = want
                pushed = self.move_tcp(side, under(a), None, cartesian=True, tol_orient=0.4,
                                       label=f"door up to {math.degrees(a):.0f} deg, again",
                                       speed=DOOR_PUSH_SPEED, fallback=False, record=pushes)
            if not pushed:
                if shut and shut():
                    # a push the door's frame stopped: it is home
                    self.log.info(f"{side} arm: the door is shut")
                    break
                # ...and failing that, on to the next step's point (the door
                # is usually well ahead of the fist by then); two in a row,
                # and the push is over
                missed += 1
                if missed < 2 and i < steps:
                    continue
                if done < 0.6 * steps:
                    return FAIL
                break
            missed = 0
            done = i
            if on_step:
                on_step(a)
            if shut and shut():
                self.log.info(f"{side} arm: the door is shut")
                break
        # Off the door the way the last push came, run backwards: with the
        # fist pressed on the shut door the fingertips sit on the face, in
        # the planning scene a hair inside the microwave's front, and a
        # straight path from there was refused in nearly every errand (the
        # planned alternative spent 13 s failing from that start state,
        # measured). Nothing is planned for the way back.
        if pushes and self.retrace(side, pushes[-1:], "off the door"):
            return OK
        # else straight back or not at all; the caller's tuck copes with it
        a = start + (end - start) * done / steps
        for back in (0.06, 0.03):
            away = tuple(p + d for p, d in zip(under(a), (-back, 0.0, -back / 3)))
            if self.move_tcp(side, away, None, cartesian=True, label="let go", fallback=False):
                break
        return OK          # the caller tucks the arm, see door()
