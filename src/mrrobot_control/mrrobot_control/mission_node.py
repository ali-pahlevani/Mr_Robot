"""The errand, as an interpreter over a YAML step list.

    ros2 run mrrobot_control mission --ros-args -p mission_file:=<yaml>

Each step is a verb with arguments; each verb is one method returning OK,
RETRY or FAIL. A RETRY is re-run up to `retries[verb]` times with the
verb's own recovery: `goto` clears the costmaps first, `find` sweeps
the head, `pick` finds the object again before grasping, `place` and `door`
simply re-approach. A step that runs out of retries ends the mission with a
per-step log of what happened.

Room travel is Nav2 (nav.py), arm motion is MoveIt (arms.py), objects come
from the colour detector through the object memory's find service; this file
only sequences them.
"""

import math
import re
import time

import rclpy
import yaml
from mrrobot_msgs.srv import FindObject
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
from webots_ros2_msgs.srv import GetBool

from mrrobot_control import arm_kinematics as k
from mrrobot_control import arms as A
from mrrobot_control.nav import (RUNUP_PER_LATERAL, Navigator, approach_path, turn_shift,
                                 wrap)

OK, RETRY, FAIL = "ok", "retry", "fail"

# How far the base backs away after letting go of the opened door: the
# fingertips end the pull at about x 1.41 and the flat door's edge is at
# 1.32, so 9 cm clears it and 12 cm clears it with the pose's error.
DOOR_RELEASE_BACK = 0.12
# Docking (see come_in / arrive): an approach wants at least MIN_RUNUP of
# straight floor before its goal, and nav.RUNUP_PER_LATERAL metres of it for
# every metre it has to move sideways; one that would need more than
# MAX_RUNUP goes via a run-up point RUNUP before the dock instead, which
# costs two turns on the spot. On the dock means within DOCK_LATERAL
# sideways and DOCK_HEADING of heading (either moves a target at 0.6 m by
# the width of a grip's IK band, measured) and no more than DOCK_PAST beyond.
RUNUP = 0.40
MIN_RUNUP = 0.20
MAX_RUNUP = 0.45
DIRECT_HEADING = math.radians(10.0)
# On the dock's line already (sideways and in heading): the base just drives
# straight along it, up to MAX_STRAIGHT either way.
ON_LINE = 0.015
ON_LINE_HEADING = math.radians(2.0)
MAX_STRAIGHT = 0.60
DOCK_LATERAL = 0.03
DOCK_HEADING = math.radians(4.0)
DOCK_PAST = 0.03
# ...and no more than DOCK_SHORT short of it: the door's wrist camera reads
# the panel and the arm reaches for what it reads, and a jar's target is in
# base_link from the live pose, so a few centimetres short cost nothing.
DOCK_SHORT = 0.05
# An object's station is judged by the camera, not the map: where the jar is
# in base_link, from detections made at rest on the dock, against the
# station's `at` -- within OBJECT_LATERAL sideways (the grip's IK band is
# 4 cm wide) and OBJECT_AHEAD either way along.
OBJECT_LATERAL = 0.025
OBJECT_AHEAD = 0.03
# A turn on the spot onto a dock's line (come_in) is made to this: at a
# target 0.85 m out, a degree of heading is 1.3 cm sideways at the hand.
TURN_ONTO = math.radians(0.8)
# A heading chosen for a way in (face_for) must have the guidance law end
# within this of the dock (the heading within what the approach's last
# small turn on the spot takes out), asking no curve tighter than
# FACE_KAPPA (a 0.5 m radius: much tighter and the odometry stops seeing
# where the base goes, see nav.KAPPA_MAX); and the run-up gets FACE_MARGIN
# extra.
FACE_LATERAL = 0.005
FACE_HEADING = math.radians(2.5)
FACE_KAPPA = 2.0
FACE_MARGIN = 0.05


class MissionNode(Node):
    def __init__(self):
        super().__init__("mrRobot_mission")
        self.declare_parameter("mission_file", "")
        # A contact probe at every release (the driver prints the
        # simulator's contact list): evidence when a place is in doubt,
        # two seconds a jar otherwise
        self.declare_parameter("probe_contacts", False)
        path = self.get_parameter("mission_file").value
        if not path:
            raise SystemExit("mission: set mission_file:=<yaml>")
        with open(path) as f:
            self.spec = yaml.safe_load(f)
        self.nav = Navigator(self)
        self.arms = A.Arms(self)
        self.find_client = self.create_client(FindObject, "/object_memory/find")
        self.reloc_client = self.create_client(Trigger, "/relocalize")
        self.objects = {}            # name -> (x, y, z) in map, from find
        self.slides = {}             # fixture -> (dx, dy) map, see slide_for
        # odom's pose in the map at the last scan match that was accepted,
        # and whether the latest one was: when it was not, the base's pose is
        # dead-reckoned from there (base_pose)
        self.anchor = None
        self.reloc_ok = True
        self._reloc_at = None        # the odometry's pose at that match
        # the colour detector's raw output, newest last: (stamp ns, name,
        # (x, y, z) map) -- see sighting()
        self._seen = []
        self.create_subscription(Detection3DArray, "/mrRobot/detections", self._on_detections, 10)
        # the last sighting of each object and where from: (odom pose, how
        # many scan matches had been applied, map point) -- see still_sighted
        self._sighted = {}
        self._relocs = 0
        self.docked = False
        self.docked_at = None        # the station the base is docked at
        self.station = None
        # the head's gaze: a map point it is kept on while the base moves
        # (the navigator's driving loops call _gaze_tick)
        self.gaze_at = None
        self.nav.on_tick = self._gaze_tick
        self.log = []
        self.door_open = False
        # The simulator's own reading of the door hinge, when it publishes
        # one: logged as evidence and used to catch a push that missed.
        self.door_truth = None
        self.create_subscription(Float32, "/ground_truth/microwave_door",
                                 lambda m: setattr(self, "door_truth", m.data), 10)
        # The simulator's contact list, logged at the moments a hand lets
        # go (the driver prints it) -- what the joint states cannot tell
        self.contacts_client = self.create_client(GetBool, "/ground_truth/contacts")
        if self.get_parameter("probe_contacts").value:
            self.arms.probe = self.probe_contacts
        # what the errand is doing, for RViz: a caption over the robot and
        # a ring on the current target
        self._markers = self.create_publisher(MarkerArray, "/mrRobot/mission/markers", 1)
        self._caption, self._prefix = "", ""
        self._target = None
        self.create_timer(0.5, self._publish_markers)

    def _on_detections(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        for det in msg.detections:
            p = det.bbox.center.position
            self._seen.append((stamp, det.id, (p.x, p.y, p.z)))
        del self._seen[:-200]

    def sighting(self, name, n=3, timeout=2.5, near=None):
        """Where the camera sees `name` NOW (map): the median of the next `n`
        detections of it, from images taken after this call, or None.

        The detector puts each detection into the map through TF as it is
        processed, so a detection is only as good as the pose estimate of
        that moment; one taken before a relocalize is in a frame that no
        longer exists, and the object memory's median mixes them for a
        second and a half (once, a stale detection moved a dock 10.8 cm).
        Detections from after the call share the estimate the arm will be
        planned from, and the localization error then cancels. `near`
        ((x, y), radius): only detections inside it count (the kitchen has
        two jam jars)."""
        since = self.get_clock().now().nanoseconds
        if near is None:
            last = self.objects.get(name)
            spec = self.spec["objects"].get(name, {}).get("near")
            if last is not None:
                near = ((last[0], last[1]), 0.20)
            elif spec:
                near = ((spec[0], spec[1]), spec[3])
        end = time.time() + timeout
        got = []
        while time.time() < end:
            got = [p for t, who, p in self._seen if t >= since and who == name
                   and (near is None
                        or math.hypot(p[0] - near[0][0], p[1] - near[0][1]) < near[1])]
            if len(got) >= n:
                break
            self.nav._spin_once()
        if len(got) < max(2, n - 1):
            return None
        xs, ys, zs = (sorted(p[i] for p in got) for i in range(3))
        mid = len(got) // 2
        self._sighted[name] = (self.nav.pose(frame="odom"), self._relocs,
                               (xs[mid], ys[mid], zs[mid]))
        return xs[mid], ys[mid], zs[mid]

    def still_sighted(self, name):
        """The last sighting of `name` if neither the base nor the estimate
        has moved since -- as good as a fresh one, without waiting for three
        more frames -- else None."""
        seen, here = self._sighted.get(name), self.nav.pose(frame="odom")
        if seen is None or here is None or seen[0] is None or seen[1] != self._relocs:
            return None
        x, y, th = seen[0]
        if (math.hypot(here[0] - x, here[1] - y) < 0.003
                and abs(wrap(here[2] - th)) < math.radians(0.2)):
            return seen[2]
        return None

    def _publish_markers(self):
        out = MarkerArray()
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id, text.action = "mission", 0, Marker.ADD
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = 2.05
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = self._caption
        out.markers.append(text)
        ring = Marker()
        ring.header.frame_id = "map"
        ring.header.stamp = text.header.stamp
        ring.ns, ring.id, ring.type = "mission", 1, Marker.CYLINDER
        ring.action = Marker.ADD if self._target else Marker.DELETE
        if self._target:
            ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = self._target
        ring.pose.orientation.w = 1.0
        ring.scale.x = ring.scale.y = 0.16
        ring.scale.z = 0.01
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = 0.2, 0.9, 1.0, 0.6
        out.markers.append(ring)
        self._markers.publish(out)

    def show(self, caption, target=None):
        """Caption for RViz, and the map point the step is about (or None)."""
        self._caption = self._prefix + caption
        self._target = None if target is None else tuple(float(v) for v in target[:3])
        self._publish_markers()

    def probe_contacts(self, tag):
        if self.contacts_client.wait_for_service(timeout_sec=0.5):
            self.say(f"probe: contacts at '{tag}' (see the driver's log)")
            fut = self.contacts_client.call_async(GetBool.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    # ------------------------------------------------------------ helpers
    def to_base(self, point):
        """A map point in base_link, from where the base is (base_pose)."""
        x, y, yaw = self.base_pose()
        dx, dy = point[0] - x, point[1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy, point[2] - k.BASE_LINK_Z)

    def resolve(self, ref):
        """A {object: ..} or {fixture: ..} reference, or a literal [x, y, z].
        A fixture comes with the slide its station chose (see slide_for)."""
        if isinstance(ref, (list, tuple)):
            return tuple(float(v) for v in ref)
        if "object" in ref:
            if ref["object"] not in self.objects:
                return None
            return self.objects[ref["object"]]
        x, y, z = (float(v) for v in self.spec["fixtures"][ref["fixture"]])
        dx, dy = self.slides.get(ref["fixture"], (0.0, 0.0))
        return x + dx, y + dy, z

    def say(self, msg):
        self.get_logger().info(msg)

    def sane(self, point):
        """A found object must be somewhere the robot could see it."""
        x, y, _ = self.base_pose()
        return math.hypot(point[0] - x, point[1] - y) < 1.6 and 0.3 < point[2] < 1.6

    def relocalize(self, station=None, where="dock"):
        """Ask the scan matcher to correct AMCL where the base stands, and
        wait for the corrected transform to be the one TF serves.

        A station may set `relocalize: false` when the map cannot be matched
        on its dock -- at the dining table the map holds the table's painted
        TOP and the lidar sees its four legs, so every fit came out at 6-8 cm
        against a 3 cm limit and the estimate was left alone anyway -- or
        `relocalize: runup` for "at the run-up point only", or `leg` for "at
        the end of the hop's straight leg only, before its last turn" (the
        dining table: facing it, from the run-up point, the scan cannot be
        matched either; facing the counters on the way there it can)."""
        if station is not None:
            mode = station.get("relocalize", True)
            if mode is False or (mode in ("runup", "leg") and where != mode):
                return False
        if not self.reloc_client.wait_for_service(timeout_sec=3.0):
            self.say("relocalize: service not available")
            return
        # nothing has moved since the last match that was accepted: it stands
        # (a goto's run-up and its dock check were often the same pose)
        here = self.nav.pose(frame="odom")
        if (self.reloc_ok and self._reloc_at is not None and here is not None
                and math.hypot(here[0] - self._reloc_at[0], here[1] - self._reloc_at[1]) < 0.003
                and abs(wrap(here[2] - self._reloc_at[2])) < math.radians(0.2)):
            return True
        x0, y0, yaw0 = self.nav.pose()
        fut = self.reloc_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if fut.result() is None:
            return False
        self.say("relocalize: " + fut.result().message)
        if not fut.result().success:
            self.reloc_ok = False
            return False
        # wait for TF to carry the correction: AMCL re-seeds on the next
        # scan, and once (measured) the station check ran on the old
        # transform and let a 13 cm error through
        m = re.search(r"moved \(([-+\d.]+), ([-+\d.]+), ([-+\d.]+) deg\)", fut.result().message)
        if m:
            dx, dy = float(m.group(1)), float(m.group(2))
            end = self.get_clock().now() + rclpy.duration.Duration(seconds=4.0)
            while self.get_clock().now() < end:
                x, y, _ = self.nav.pose()
                if math.hypot(x - x0 - dx, y - y0 - dy) < 0.02:
                    break
                self.nav.spin_for(0.1)
            else:
                self.say("relocalize: TF did not take the correction in 4 s")
        self.nav.spin_for(0.5)
        self.anchor = self.nav.odom_in_map()
        self.nav.reset_slip()
        self.reloc_ok = True
        self._reloc_at = self.nav.pose(frame="odom")
        self._relocs += 1
        return True

    def base_pose(self):
        """Where the base is, in the map: AMCL's estimate, unless the latest
        scan match was refused -- then dead reckoning from the last one that
        was accepted (nav.dead_reckoning). By the dining table AMCL drifted
        15 cm after a hop while the refused match saw it, and the dock and
        the place built on AMCL's word put the jam 12 cm off its spot
        (measured)."""
        if self.anchor is not None and (not self.reloc_ok or self.dead_reckons()):
            return self.nav.dead_reckoning(self.anchor)
        return self.nav.pose()

    def dead_reckons(self, station=None):
        """Is the current station (or `station`) one whose dock the scan
        cannot be matched on, so that AMCL is not to be believed there?"""
        if station is None:
            station = self.spec["stations"].get(self.station or "", {})
        return station.get("relocalize", True) in (False, "runup", "leg")

    def build_scene(self):
        """The furniture, in base_link from the live pose.

        A box that the pose estimate would put INSIDE the base is pushed out
        to the bumper: AMCL was 12 cm off once after a creep that ended with
        the nose on the plinth, the counter landed inside base_link's own
        collision box, and every plan failed with a start state in
        collision. What matters about a counter is its top, not its exact
        front.

        Ten boxes, each a publish and a poll of the monitored scene, and the
        verbs ask for them after every dock, every door and every creep. The
        furniture does not move: only the base does, so a rebuild from a pose
        the scene was already built at is skipped."""
        x, y, yaw = self.base_pose()
        where = (round(x, 3), round(y, 3), round(yaw, 3))
        if where == getattr(self, "_scene_at", None):
            return
        self._scene_at = where
        quat = (0.0, 0.0, math.sin(-yaw / 2), math.cos(-yaw / 2))
        bumper = k.BUMPER_X + 0.03
        for name, box in self.spec.get("scene", {}).items():
            cx, cy, cz = self.to_base(box["center"])
            sx, sy, sz = box["size"]
            # only clamp when the box is ahead, low enough to meet the chassis,
            # and would overlap it (a worktop slab above the base is left alone)
            if (cz - sz / 2 < 0.45 and abs(cy) < sy / 2 + 0.3
                    and cx - sx / 2 < bumper < cx + sx / 2):
                new_front = bumper
                back = cx + sx / 2
                cx, sx = (new_front + back) / 2, back - new_front
                self.say(f"scene: {name} front pushed out to the bumper")
            self.arms.add_box(name, (sx, sy, sz), (cx, cy, cz), quat)

    def clear_scene(self):
        for name in self.spec.get("scene", {}):
            self.arms.remove_object(name)
        self.door_scene(None)
        self._scene_at = None       # build_scene's cache: the boxes are gone

    def door_scene(self, angle):
        """The microwave door in the planning scene at hinge angle `angle`
        (None removes it): the dropped door and the bar under it are what a
        jar going into the cavity has to clear."""
        if angle is None:
            for name in ("microwave_door", "microwave_bar"):
                self.arms.remove_object(name)
            return
        hinge = tuple(self.spec["fixtures"]["microwave_hinge"])
        handle = tuple(self.spec["fixtures"]["microwave_handle"])
        _, _, yaw = self.base_pose()
        tilt = A.quat_mul((0.0, 0.0, math.sin(-yaw / 2), math.cos(-yaw / 2)),
                          A.quat_about_y(-angle))
        # the slab: 2.5 cm thick in front of the hinge plane, 17.9 cm tall,
        # modelled twice as thick on its outer side (the underside, once
        # open): a jar hangs a centimetre or two lower in the pads than
        # its box, and the swing up to the hover, planned to clear the
        # real slab by a hair, took the jar's top through the door's tip
        # (measured). The bar box is padded a centimetre all round too.
        slab = A.door_point(hinge, (-0.025, 0.0, 0.0895), angle)
        self.arms.add_box("microwave_door", (0.050, 0.462, 0.179), self.to_base(slab), tilt)
        bar = A.door_handle_at(handle, hinge, angle)
        self.arms.add_box("microwave_bar", (0.030, 0.408, 0.030), self.to_base(bar), tilt)

    # ------------------------------------------------------------ stations
    def dock_pose(self, station):
        """(goal, target): the base_link pose that puts the station's target
        where the station's `at` says it should be in base_link -- the jar
        0.77 m ahead and 0.37 m to the left, the handle 0.70 m ahead -- and
        the target itself (map). An object is taken from what the camera has
        seen of it, else from where the errand expects it (`near`)."""
        ref = station["target"]
        target = self.resolve(ref)
        if target is None and "object" in ref:
            near = self.spec["objects"].get(ref["object"], {}).get("near")
            if near is not None:
                target = tuple(float(v) for v in near[:3])
        if target is None:
            return None, None
        ax, ay = (float(v) for v in station["at"])
        yaw = station.get("_heading", math.radians(float(station.get("heading", 0.0))))
        c, s = math.cos(yaw), math.sin(yaw)
        return (target[0] - (c * ax - s * ay), target[1] - (s * ax + c * ay), yaw), target

    def slide_for(self, station):
        """A station whose target is a bar -- the microwave's handle, 41 cm
        long, gripped anywhere 6 cm or more from its ends -- may take it
        anywhere within `slide` [lo, hi] metres (to the robot's left) of its
        middle. It takes the point that puts the dock in line with where the
        base already is, so the move there has as little sideways in it as
        the bar allows: from the honey's dock, none -- the base only backs
        out; from the cavity's, none either, once the base has turned back
        to face the door (in line with where the turn will leave it,
        turn_error). Chosen as come_in sets off, and again after a turn;
        the arm skill then grips there."""
        ref = station["target"]
        if "fixture" not in ref:
            return
        self.slides.pop(ref["fixture"], None)
        if "slide" not in station:
            return
        lo, hi = (float(v) for v in station["slide"])
        goal, _ = self.dock_pose(station)
        _, lateral = self.turn_error(goal)
        off = max(lo, min(hi, lateral))
        yaw = goal[2]
        self.slides[ref["fixture"]] = (-math.sin(yaw) * off, math.cos(yaw) * off)
        self.say(f"{ref['fixture']}: taken {off * 100:+.1f} cm along from its middle")

    def face_for(self, station, label=""):
        """A station that may be faced from a range of headings (`headings`:
        [lo, hi], degrees in the map) takes the heading whose dock one
        forward approach reaches from where the base stands, backed
        straight out along its heading as little as possible first
        (`_runup`), by the approach's own guidance (nav.approach_path) and
        with every curve gentle enough for the odometry to follow. Of equal
        run-ups, the heading nearest square on.

        The microwave's cavity, from where the door was let go of: it has
        to be 0.36 m to the LEFT for the left hand, and the door's handle
        0.1 m to the RIGHT for the right one, and facing the counter square
        on that was two hops each way, 45 s of shuffling sideways
        (measured). Faced 30-odd degrees right, the cavity is where the
        hand wants it from nearly the same spot. The base does not turn on
        the spot to get there: a 30 deg turn beside the counter carried it
        9 cm, 5 cm more than the pivot model says, and once facing the
        counter at that angle the scan matcher cannot see the north counter
        (it is behind the lidar's blind half) and slid 7-9 cm along the
        east one while scoring a good fit (measured). So the scan is matched
        facing square on, before the approach, and the dock is reached on
        odometry (the station's `relocalize: runup`).

        None in range: the station's own `heading`, and come_in's usual way
        in."""
        station.pop("_heading", None)
        station.pop("_runup", None)
        if "headings" not in station:
            return
        lo, hi = sorted(math.radians(float(v)) for v in station["headings"])
        room = min(MAX_RUNUP, float(station.get("runup", MAX_RUNUP)))
        x, y, th = self.base_pose()

        def miss(goal, back):
            """How far from a good end an approach from `back` metres back
            comes (0 when it ends on the dock), from the guidance law."""
            along, lateral, psi, tightest = approach_path(
                x - back * math.cos(th), y - back * math.sin(th), th, *goal)
            return max(abs(lateral) / FACE_LATERAL, abs(psi) / FACE_HEADING,
                       tightest / FACE_KAPPA, 1.0) - 1.0

        best, nearest = None, None
        n = max(1, int(round(math.degrees(hi - lo) / 2.0)))
        for i in range(n + 1):
            h = hi - (hi - lo) * i / n          # from the squarest
            station["_heading"] = h
            goal, _ = self.dock_pose(station)
            for j in range(int(room / 0.03) + 1):   # the least run-up that does
                b = 0.03 * j
                if best is not None and b >= best[1]:
                    break
                m = miss(goal, b)
                if nearest is None or m < nearest[2]:
                    nearest = (h, b, m)
                if m == 0.0:
                    best = (h, b)
                    break
        if best is None:
            station.pop("_heading", None)
            self.say(f"{label}: no heading in {station['headings']} has a way in from here "
                     f"(nearest: {math.degrees(nearest[0]):+.0f} deg from {nearest[1]:.2f} m, "
                     f"{nearest[2]:.1f} over)")
            return
        station["_heading"] = best[0]
        # a margin over the guidance law's ideal (the real one lags)
        station["_runup"] = min(room, best[1] + FACE_MARGIN) if best[1] > 0 else 0.0
        self.say(f"{label}: faced from {math.degrees(best[0]):+.0f} deg, one curve in after "
                 f"backing out {station['_runup'] * 100:.0f} cm")

    def turn_error(self, goal, here=None):
        """(along, lateral) off `goal`, in its frame, of where base_link will
        be once the base has turned on the spot to the goal's heading: a
        turn carries it round the pivot (nav.turn_shift), 5 cm in a 30 deg
        turn."""
        x, y, th = here or self.base_pose()
        gx, gy, gyaw = goal
        sx, sy = turn_shift(th, wrap(gyaw - th))
        x, y = x + sx, y + sy
        c, s = math.cos(gyaw), math.sin(gyaw)
        return c * (x - gx) + s * (y - gy), -s * (x - gx) + c * (y - gy)

    def pose_error(self, goal):
        """(along, lateral, heading) of the live pose off `goal`, in the goal's
        own frame: along > 0 is past it."""
        x, y, th = self.base_pose()
        gx, gy, gyaw = goal
        c, s = math.cos(gyaw), math.sin(gyaw)
        return (c * (x - gx) + s * (y - gy), -s * (x - gx) + c * (y - gy),
                (th - gyaw + math.pi) % (2 * math.pi) - math.pi)

    def object_of(self, station):
        """The object a station is for, or None for a fixture's."""
        return station["target"].get("object") if isinstance(station["target"], dict) else None

    def object_error(self, station, point):
        """(ahead, left) of an object seen at `point` (map) off the station's
        `at`, in base_link from the live pose."""
        bx, by, _ = self.to_base(point)
        ax, ay = (float(v) for v in station["at"])
        return bx - ax, by - ay

    def come_in(self, station, label, runup=None):
        """Get onto the station's dock with ONE forward approach: straight along
        the dock's line when the base is on it, or a turn on the spot away
        from it; else, for a dock with `headings`, the way in face_for
        chose (straight back, one curve); else back straight out only as
        far as the approach needs to take out the sideways error; else hop
        to a run-up point on the line (the turns happen there), and again
        if the hop did not land within reach of it. The estimate is made
        good by a scan match before the base moves and at the run-up point
        (and where the hop's leg ends, unless `leg_match: false`), the goal
        is taken afresh after each -- a jar's from what the camera sees
        then -- and approached on odometry. Where the dock cannot be scan
        matched (`relocalize: runup`, or a match refused) it is reached by
        dead reckoning from the last match accepted. True when the approach
        was made.

        Every relocalize comes BEFORE the goal is taken: a goal from a
        detection made in one estimate and driven to in another is off by
        whatever the relocalize moved (once 10 cm, and three re-approaches
        to chase it)."""
        runup = float(station.get("runup", RUNUP)) if runup is None else runup
        goal, _ = self.dock_pose(station)
        if goal is None:
            return False
        # never with an arm out (a retry comes here straight from a skill
        # that failed), and a jar in hand pinched lightly while the base
        # drives (see arm_kinematics.FINGER_CARRY), firmly again before the
        # arm works
        if not self.tuck_all():
            return False
        self.grip(firm=False)
        # the estimate made good where the base stands, before it moves: a
        # hop is aimed through it, and by the dining table, where the scan
        # cannot be matched, it is all the dead reckoning has to go on --
        # unless the base stands on a dock that it dead-reckoned its way
        # onto because the scan cannot be matched there (facing the
        # microwave's cavity a match slid 7-9 cm and still scored well,
        # measured): then it dead-reckons on from the last good match
        if self.docked and self.dead_reckons(self.spec["stations"].get(self.docked_at or "", {})):
            self.reloc_ok = False
        else:
            self.relocalize()
        # where along a bar, and from which heading, the dock is taken: both
        # chosen from where the base now stands
        self.slide_for(station)
        self.face_for(station, label)
        goal, _ = self.dock_pose(station)
        along, lateral, dyaw = self.pose_error(goal)
        # A turn on the spot that leaves the base on the dock's line (the
        # cavity's, faced from where the door was opened; the door's, turned
        # back to from the cavity's): turned, the estimate made good, and
        # the choice made again from there -- the pivot is good to a few
        # centimetres, so a second turn, if any, is a degree or two.
        tol = TURN_ONTO if "headings" in station else ON_LINE_HEADING
        for _ in range(3):
            t_along, t_lateral = self.turn_error(goal)
            if ("_runup" in station or abs(dyaw) <= tol or abs(t_lateral) > ON_LINE
                    or abs(t_along) > MAX_STRAIGHT):
                break
            _, _, heading = self.nav.pose(frame="odom")
            self.nav.turn_to(wrap(heading - dyaw), tol=0.6 * tol, frame="odom")
            self.docked = False
            self.relocalize(station, where="runup")
            self.slide_for(station)
            self.face_for(station, label)
            goal, _ = self.dock_pose(station)
            along, lateral, dyaw = self.pose_error(goal)
        gx, gy, gyaw = goal
        if ("_runup" not in station and abs(lateral) <= ON_LINE and abs(dyaw) <= ON_LINE_HEADING
                and abs(along) <= MAX_STRAIGHT):
            # on the dock's line: straight along it, forwards or back, and
            # no run-up (the honey's dock to the door's is 18 cm straight
            # back, once the door station has slid into line)
            if along < -0.005:
                self.nav.creep(lambda: False, -along, speed=0.08)
            elif along > 0.005:
                self.nav.back_out(along)
            else:
                return True
            self.docked = False
            self.relocalize(station, where="runup")
            # and the last of it, now that the estimate is made good: a back-out
            # stops on a moving estimate and once ended 4 cm short
            along, lateral, dyaw = self.pose_error(goal)
            if along > 0.015:
                self.nav.back_out(along, speed=0.08)
            elif along < -0.015:
                self.nav.creep(lambda: False, -along, speed=0.08)
            return True
        # the run-up the station has room for bounds what an approach can
        # take out sideways: backed straight out, `back_room` (the jam's
        # 0.40 m keeps the rear 9 cm off the dining chairs' backs), else
        # the spin's `runup`
        room = min(MAX_RUNUP, float(station.get("back_room", station.get("runup", MAX_RUNUP))))
        hops = 0
        if "_runup" in station:
            # the way in face_for chose: straight back, then one curve
            if station["_runup"] > 0.005:
                self.nav.back_out(station["_runup"])
            self.docked = False
        while "_runup" not in station:
            need = max(MIN_RUNUP, abs(lateral) * RUNUP_PER_LATERAL)
            if abs(dyaw) < DIRECT_HEADING and need <= room:
                # (a back-out of a couple of centimetres is not worth its
                # scan match: on 3 cm less the approach still ends within a
                # centimetre, simulated with its slide)
                if along > -need + 0.03:
                    # back out along the current heading (the goal's, near enough)
                    self.nav.back_out(along + need)
                    self.docked = False
                    hops = 0             # moved since the last relocalize
                break
            if hops == 2:
                # two hops that did not land within reach of the line: come
                # in from here anyway, and let the dock check decide
                break
            if self.docked:
                self.nav.back_out(0.20)      # room to turn before the hop
                self.docked = False
            ax, ay = gx - runup * math.cos(gyaw), gy - runup * math.sin(gyaw)
            self.say(f"{label}: via ({ax:.2f}, {ay:.2f}), {runup:.2f} m before the dock")
            # a scan match where the hop's straight leg ends, to aim its last
            # turn -- unless the station says the map is not to be trusted
            # there (`leg_match: false`)
            leg = None if station.get("leg_match", True) is False else (
                lambda: self.relocalize(station, where="leg"))
            if not self.nav.navigate_to(ax, ay, gyaw, before_turn=leg):
                return False
            hops += 1
            self.relocalize(station, where="runup")
            goal = self.refresh_goal(station, goal)
            gx, gy, gyaw = goal
            along, lateral, dyaw = self.pose_error(goal)
        self.docked = False
        if hops == 0:
            self.relocalize(station, where="runup")
            goal = self.refresh_goal(station, goal)
        if self.object_of(station) is None and self.anchor is not None and (
                not self.reloc_ok or self.dead_reckons(station)):
            # a fixture, and the scan cannot be matched here: dead reckoning
            odom_goal = self.nav.odom_target(*goal, self.anchor)
        else:
            # a jar was seen in AMCL's estimate as it is now, and so is the
            # odometry's frame to drive to it in
            odom_goal = self.nav.map_to_odom(*goal)
        if odom_goal is None:
            self.nav.approach(*goal)
        else:
            self.nav.approach(*odom_goal, frame="odom")
        return True

    def refresh_goal(self, station, goal):
        """The dock again after a relocalize: for an object, from what the
        camera sees now, in the corrected estimate; a fixture's does not
        move."""
        name = self.object_of(station)
        if name is None:
            return goal
        seen = self.sighting(name)
        if seen is not None and self.sane(seen):
            self.objects[name] = seen
        return self.dock_pose(station)[0]

    def home(self, side):
        """An arm home after a skill that failed: by the planner (bring_back),
        else walked home joint by joint with the column up first
        (arms.free). A reach that hit the counter once left the wrist
        camera inside it as far as the planner was concerned, every plan
        after that was refused a start state in collision, and the retry
        drove the base with the arm still out (measured)."""
        if self.arms.bring_back(side):
            return True
        if self.arms.holding[side]:
            return self.arms.carry(side)
        return self.arms.free(side)

    def grip(self, firm):
        for side in ("left", "right"):
            self.arms.hold(side, firm)

    def on_dock(self, station, label):
        """Is the base where the station wants it? An object's station asks
        the camera (object_error); a fixture's asks the corrected estimate.
        Returns (ok, why)."""
        name = self.object_of(station)
        if name is not None:
            seen = self.sighting(name)
            if seen is not None and self.sane(seen):
                self.objects[name] = seen
                ahead, left = self.object_error(station, seen)
                _, _, dyaw = self.pose_error(self.dock_pose(station)[0])
                # further than wanted with the nose already at the plinth is
                # as near as it gets: the grip is planned from what is seen,
                # and has IK to 6 cm beyond `at`
                far_ok = 0 < ahead <= 0.06 and self.nav.nose_on_something()
                ok = (abs(left) <= OBJECT_LATERAL and (abs(ahead) <= OBJECT_AHEAD or far_ok)
                      and abs(dyaw) <= DOCK_HEADING)
                return ok, (f"the {name} is {ahead * 100:+.1f} cm ahead / {left * 100:+.1f} cm "
                            f"left of where the grip wants it, heading "
                            f"{math.degrees(dyaw):+.1f} deg")
            self.say(f"{label}: the camera does not see the {name}; going by the map")
        if self.dead_reckons(station):
            # A dock the scan cannot be matched on (the microwave's cavity,
            # faced at an angle): AMCL's estimate there wanders while the
            # odometry the approach drove on is good to a few millimetres
            # (measured), so the pose is dead-reckoned from the run-up's
            # match (base_pose).
            goal, _ = self.dock_pose(station)
            along, lateral, dyaw = self.pose_error(goal)
            return self.docked_well(along, lateral, dyaw), (
                f"{lateral * 100:+.1f} cm off the line, {along * 100:+.1f} cm along it, "
                f"{math.degrees(dyaw):+.1f} deg (dead reckoning)")
        self.relocalize(station, where="dock")
        goal, _ = self.dock_pose(station)
        along, lateral, dyaw = self.pose_error(goal)
        ok = self.docked_well(along, lateral, dyaw)
        return ok, (f"{lateral * 100:+.1f} cm off the line, {along * 100:+.1f} cm along it, "
                    f"{math.degrees(dyaw):+.1f} deg")

    @staticmethod
    def docked_well(along, lateral, dyaw):
        """The one test of a pose error against the dock, for the goto's check
        and a verb's alike (they used to differ by a millimetre, and a dock
        the goto had accepted 4.0 cm short was re-approached by the door)."""
        return (abs(lateral) <= DOCK_LATERAL and abs(dyaw) <= DOCK_HEADING
                and -DOCK_SHORT <= along <= DOCK_PAST)

    def arrive(self, station, label):
        """After an approach: on the dock, or come in once or twice more.
        True when on it."""
        for i in range(3):
            ok, why = self.on_dock(station, label)
            self.say(f"{label}: {why}" + ("" if ok else "; coming in again"))
            if ok:
                return True
            if i < 2:
                self.come_in(station, label)
        return False

    def _gaze_tick(self):
        if self.gaze_at is not None:
            self.arms.gaze(self.gaze_at)

    def look_at(self, point):
        """Keep the head on a map point from now on (while the base moves, the
        navigator's loops re-aim it); None stops tracking."""
        self.gaze_at = point
        self.arms._gaze_sent = None
        if point is not None:
            self.arms.gaze(point)

    # ------------------------------------------------------------ verbs
    def do_posture(self, step):
        self.show(f"posture {step['name']}")
        self.look_at(None)
        for side in ("left", "right"):
            if not self.arms.posture(side, step["name"]):
                # an arm left somewhere the planner cannot start from (a
                # previous run's wreck) is walked out joint by joint first
                if self.arms.holding[side] or not self.arms.free(side, step["name"]):
                    return FAIL
                if not self.arms.posture(side, step["name"]):
                    return FAIL
        self.arms.look(-0.15, 0.0)
        return OK

    def tuck_all(self):
        """Both arms home (a hand with a jar in it level in front of its
        shoulder) and the column at mid travel -- the state every arm skill
        starts from and returns to, and the one the base drives in. Arm and
        column move together (arms.bring_back); an arm MoveIt cannot bring
        back is walked home joint by joint."""
        for s in ("left", "right"):
            if self.arms.bring_back(s):
                continue
            if self.arms.holding[s]:
                self.say(f"tuck: could not bring the {s} arm to the carry pose")
                return False
            if not self.arms.free(s):
                self.say(f"tuck: could not tuck the {s} arm")
                return False
        return self.arms.lift_to(0.0)

    def lift_for(self, side, target_map, shoulder_above=A.JAR_SHOULDER_ABOVE):
        """The column height that puts the shoulder `shoulder_above` metres
        above the target (a jar's grip height by default)."""
        x, y, yaw = self.base_pose()
        return k.best_lift_for(side, x, y, yaw, target_map, shoulder_above)

    def do_goto(self, step, attempt):
        name = step["station"]
        station = self.spec["stations"][name]
        # never drive with an arm out: a hand left in the microwave anchored
        # the base to the counter once (measured: the turn gave up at 20 deg)
        if not self.tuck_all():
            return FAIL
        if attempt > 0:
            # A start "in lethal space" (the base nosed into something
            # the map paints solid) is left behind before the next try
            self.nav.clear_costmaps()
            self.nav.back_out(0.20)
            self.docked = False
        # the station from here on is the one being docked at (base_pose asks
        # it whether AMCL can be believed there)
        self.station = name
        goal, target = self.dock_pose(station)
        if goal is None:
            self.say(f"goto {name}: nothing to go to")
            return FAIL
        self.show(f"goto {name}", (goal[0], goal[1], 0.0))
        # eyes on what the station is for, all the way there: an object is
        # then in view, and seen from the run-up point, before the approach
        self.look_at(target)
        if not self.come_in(station, name):
            return RETRY
        if not self.arrive(station, name):
            return RETRY
        self.docked, self.docked_at = True, name
        self.build_scene()
        if self.door_open:
            self.door_scene(A.DOOR_OPEN)
        return OK

    def do_find(self, step, attempt):
        name = step["object"]
        # an object the errand knows the whereabouts of (two jam jars, one
        # by the microwave): the memory looks there first, and only
        # detections near there count
        near = self.spec["objects"].get(name, {}).get("near")
        self.show(f"find {name}", near)
        if not self.find_client.wait_for_service(timeout_sec=10.0):
            self.say("find: no object memory service")
            return FAIL
        if attempt > 0:
            # A retry from exactly where the last sweep saw nothing tends to
            # see nothing again (once four full sweeps, the jam jar standing
            # where it always stands). Ten centimetres further back the head
            # looks at the jar's lid from a new angle; the pick comes back in.
            self.nav.back_out(0.10)
        req = FindObject.Request()
        req.colour = name
        req.max_age = 2.0 if attempt == 0 else 0.6    # a retry wants a fresh look
        req.sweep = True
        if near:
            req.near.x, req.near.y, req.near.z = (float(v) for v in near[:3])
            req.near_radius = float(near[3])
        fut = self.find_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)
        res = fut.result()
        self.arms._gaze_sent = None        # the memory may have moved the head
        if res is None or not res.found:
            self.say(f"find: {name} not found" + ("" if res is None else f" ({res.message})"))
            return RETRY
        p = res.pose.pose.position
        if not self.sane((p.x, p.y, p.z)):
            self.say(f"find: {name} reported at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                     "which is not somewhere the camera could see; ignoring")
            return RETRY
        self.objects[name] = (p.x, p.y, p.z)
        self.say(f"find: {res.message}")
        self.show(f"find {name}", self.objects[name])
        self.look_at(self.objects[name])
        return OK

    def ready_at(self, name, hand):
        """The base on the dock for the current station, or brought onto it:
        a retry, or a find from further back, can leave it elsewhere."""
        station = self.spec["stations"].get(self.station or "")
        if not station:
            return True
        goal, _ = self.dock_pose(station)
        if goal is None:
            return True
        if self.docked and self.dead_reckons(station):
            # nothing has moved since the goto docked, and on a dock the map
            # does not fit AMCL's word is worse than the approach's (at the
            # table it said 13 cm, and the base came in again for nothing)
            return True
        along, lateral, dyaw = self.pose_error(goal)
        if self.docked and self.docked_well(along, lateral, dyaw):
            return True
        self.say(f"{hand} hand: the base is {along * 100:+.1f} cm / {lateral * 100:+.1f} cm off "
                 f"the dock for the {name}; coming in")
        if not (self.come_in(station, self.station) and self.arrive(station, self.station)):
            return False
        self.docked, self.docked_at = True, self.station
        self.build_scene()
        if self.door_open:
            self.door_scene(A.DOOR_OPEN)
        return True

    def do_pick(self, step, attempt):
        name, side = step["object"], step["hand"]
        self.show(f"pick {name} ({side} hand)", self.objects.get(name))
        if name not in self.objects or attempt > 0:
            r = self.do_find({"object": name, "hand": side}, attempt)
            if r != OK:
                return r
        if not self.ready_at(name, side):
            return RETRY
        # A FRESH detection from the final pose, right before the grasp: the
        # pads have 2 cm of lateral slack round a jar, the localization jumps
        # by that much whenever the base moves, and a detection and a grasp
        # taken from the same pose cancel the localization error exactly
        # (measured: 1-2 cm between detection and truth in base_link). The
        # head has been on the jar since the goto; the dock check's sighting
        # is such a one if nothing has moved since.
        fresh = self.still_sighted(name) or self.sighting(name)
        if fresh is None or not self.sane(fresh):
            r = self.do_find({"object": name, "hand": side}, attempt=1)
            if r != OK:
                return r
            fresh = self.objects[name]
        self.objects[name] = fresh
        target_map = fresh
        self.show(f"pick {name} ({side} hand)", target_map)
        spec = self.spec["objects"].get(name, {})
        result = self.arms.pick(side, self.to_base(target_map), self.lift_for(side, target_map),
                                radius=spec.get("radius", 0.045), height=spec.get("height", 0.115))
        if result == A.OK:
            self.look_at(None)
            if not self.arms.bring_back(side):
                self.say(f"pick: holding {name} but could not bring it home; trying once more")
                self.arms.carry(side)
            return OK
        self.home(side)
        if result == A.MISSED:
            del self.objects[name]        # it is not where we thought
        return RETRY

    def do_place(self, step, attempt):
        side = step["hand"]
        target_map = self.resolve({"fixture": step["at"]} if isinstance(step["at"], str)
                                  else step["at"])
        self.show(f"place at {step['at']} ({side} hand)", target_map)
        # a jar dropped on the way (measured: an aborted hover opened the
        # hand's grip) ends the errand here, not after two more attempts
        # at putting down nothing
        if not self.arms.holding[side] or not self.arms.closed_on_something(side):
            self.say(f"place: the {side} hand is empty")
            return FAIL
        if not self.ready_at(str(step["at"]), side):
            return RETRY
        self.grip(firm=True)
        self.look_at(target_map)
        shoulder_above = float(step.get("shoulder_above", A.JAR_SHOULDER_ABOVE))
        approach = tuple(step.get("approach", [-0.15, 0, 0]))
        lift = self.lift_for(side, target_map, shoulder_above)
        # retreat: how the open hand leaves (arms.place) -- "arm" straight
        # back, "retrace" by the joint path it came in on (out of the
        # microwave), "base" not at all: the base backs straight out
        retreat = step.get("retreat", "arm")
        lead_in = step.get("lead_in")
        if lead_in is not None:
            lead_in = tuple(float(v) for v in lead_in)
        result = self.arms.place(side, self.to_base(target_map), lift,
                                 approach, above=float(step.get("above", 0.05)),
                                 retreat=retreat, up=float(step.get("hover_lift", 0.0)),
                                 lead_in=lead_in)
        if result == A.OK and retreat == "base":
            # 0.40: at 0.30 the fingertips stopped a centimetre from the
            # open door's bar in the planning scene and the planner spent
            # 12 s on a start state it called in collision (measured).
            self.nav.back_out(float(step.get("back_out", 0.40)))
            self.docked = False
            # The boxes are in base_link from where the base stood: redo them
            # from here. Left as they were, the arm's way home put the wrist
            # into the stale microwave_top box, every plan from there was
            # refused a start state in collision, and the next goto spent 20 s
            # walking the arm home joint by joint.
            self.build_scene()
            if self.door_open:
                self.door_scene(A.DOOR_OPEN)
        self.look_at(None)
        if result == A.OK:
            self.arms.bring_back(side)
            return OK
        self.home(side)
        return RETRY

    def do_door(self, step, attempt):
        side = step["hand"]
        opening = bool(step["open"])
        # the handle where the station took it (slide_for), and the hinge --
        # an axis along y -- taken at the same y, so every point the skills
        # swing round it (the grip, the fist under the open door) is there
        handle = self.resolve({"fixture": "microwave_handle"})
        hx, _, hz = self.spec["fixtures"]["microwave_hinge"]
        hinge = (float(hx), handle[1], float(hz))
        self.show(f"{'open' if opening else 'close'} the door ({side} hand)", handle)
        # the goto has put the handle where the door wants it (the station's
        # `at`); a retry may not have
        if not self.ready_at("door", side):
            return RETRY
        # The other hand's jar keeps the light carry pinch while this one
        # pulls. The door comes open with a jolt, and under the firm 98 N
        # pinch the jolt twice collapsed the spring fingers past their stop
        # in one physics step and flung the jar across the kitchen (measured,
        # 2 errands in 33); 29 N holds it through every drive and turn.
        self.grip(firm=False)
        self.look_at(handle)
        self.build_scene()
        # the door itself is never in the scene while a hand works on it
        self.door_scene(None)
        if opening:
            result = self.arms.door(side, self.to_base(handle), self.to_base(hinge), True,
                                    self.lift_for(side, handle, A.DOOR_SHOULDER_ABOVE),
                                    opened=lambda: (self.door_truth is not None
                                                    and self.door_truth > 1.2))
        else:
            bar = A.door_handle_at(handle, hinge, A.DOOR_OPEN)

            def progress(a):
                if self.door_truth is not None:
                    self.say(f"door: pushed to {math.degrees(a):.0f} deg, "
                             f"simulator reads {math.degrees(self.door_truth):.0f} deg")
            result = self.arms.door_close(side, self.to_base(handle), self.to_base(hinge),
                                          self.lift_for(side, bar, A.DOOR_CLOSE_SHOULDER_ABOVE),
                                          on_step=progress,
                                          shut=lambda: (self.door_truth is not None
                                                        and self.door_truth < 0.03))
        want = (lambda a: a > A.DOOR_OPEN - 0.3) if opening else (lambda a: a < 0.03)

        def settle(seconds):
            end = time.time() + seconds
            while time.time() < end:
                self.nav.spin_for(0.1)
                if self.door_truth is not None and want(self.door_truth):
                    return

        self.look_at(None)
        if opening:
            # The open hand is under the door (arms.door lets go by opening
            # the fingers, the door resting on the lower one): the base backs
            # away until the fingertips are behind the flat door's edge, the
            # finger slides out from under it, and the door settles onto its
            # stop by itself -- all before the arm moves, so the tuck never
            # meets a door still on its way down.
            self.nav.back_out(DOOR_RELEASE_BACK)
            self.docked = False
            settle(2.0)
            self.build_scene()        # the base has moved (see do_place)
            self.arms.bring_back(side)
        else:
            self.arms.bring_back(side)
            settle(2.0)
        if self.door_truth is not None:
            self.say(f"door: simulator reads the hinge at {math.degrees(self.door_truth):.0f} deg")
            settled = self.door_truth > 1.2 if opening else self.door_truth < 0.1
            if not settled:
                self.say("door: not where it should be; retrying")
                return RETRY
            if result != A.OK:
                # a hand that missed a handle that is no longer there: the
                # previous attempt's door had already fallen open
                self.say("door: the skill gave up but the door is where it should be")
        elif result != A.OK:
            return RETRY
        self.door_open = opening
        if opening:
            self.door_scene(A.DOOR_OPEN)
        return OK

    # ------------------------------------------------------------ the loop
    def check_heading(self):
        """AMCL's heading against the odometry's, before anything moves. The
        map is the Webots world's frame and the EKF takes its heading from
        the IMU, which is the world's too, so map->odom should turn by next
        to nothing. Yet about one bring-up in five AMCL's first estimate comes
        out a quarter turn out (its initial pose goes in before the simulator
        steps), and once the base drove south believing it drove east and
        never saw the honey. The scan relocalizer watches for it at start-up
        too. It is re-seeded here, where the base
        stands, with the odometry's heading."""
        import tf2_ros
        from mrrobot_control.nav import yaw_of
        for _ in range(50):
            try:
                tf = self.nav._tf.lookup_transform("map", "odom", rclpy.time.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self.nav.spin_for(0.1)
        else:
            return
        turn = yaw_of(tf.transform.rotation)
        if abs(turn) < math.radians(5.0):
            return
        x, y, _ = self.nav.pose()
        _, _, heading = self.nav.pose(frame="odom")
        self.say(f"localization: AMCL is {math.degrees(turn):+.0f} deg out of line with the "
                 f"odometry; re-seeding it at ({x:.2f}, {y:.2f}, "
                 f"{math.degrees(heading):+.0f} deg)")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(heading / 2)
        msg.pose.pose.orientation.w = math.cos(heading / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
        msg.pose.covariance[35] = math.radians(2.0) ** 2
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        end = time.time() + 6.0
        while time.time() < end:
            pub.publish(msg)
            self.nav.spin_for(0.5)
            try:
                tf = self.nav._tf.lookup_transform("map", "odom", rclpy.time.Time())
                if abs(yaw_of(tf.transform.rotation)) < math.radians(2.0):
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        self.relocalize()

    def wait_for_stack(self, timeout=240.0):
        """Block until what the verbs need is up: the eight controllers
        active, move_group, Nav2 and the object memory answering. The
        launch file starts this node alongside all of them (mission:=true),
        and the first step is an arm posture that would otherwise fail
        before move_group exists; the stack takes about 15 s to come up.
        Sim time may not be ticking yet, so the deadline is wall-clock."""
        from controller_manager_msgs.srv import ListControllers
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from moveit_msgs.srv import GetStateValidity
        needed = {"joint_state_broadcaster", "base_controller", "lift_controller",
                  "head_controller", "left_arm_controller", "right_arm_controller",
                  "left_gripper_controller", "right_gripper_controller"}
        cm = self.create_client(ListControllers, "/controller_manager/list_controllers")
        mg = self.create_client(GetStateValidity, "/check_state_validity")
        nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        end = time.time() + timeout
        said = set()
        while time.time() < end:
            active = set()
            if cm.wait_for_service(timeout_sec=1.0):
                fut = cm.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
                if fut.result() is not None:
                    active = {c.name for c in fut.result().controller if c.state == "active"}
            missing = []
            if not needed <= active:
                missing.append(f"controllers ({len(needed & active)}/{len(needed)} active)")
            if not mg.wait_for_service(timeout_sec=0.5):
                missing.append("move_group")
            if not nav.wait_for_server(timeout_sec=0.5):
                missing.append("Nav2")
            if not self.find_client.wait_for_service(timeout_sec=0.5):
                missing.append("object memory")
            if not missing:
                self.say("mission: the stack is up")
                nav.destroy()
                return True
            key = tuple(missing)
            if key not in said:
                said.add(key)
                self.say("mission: waiting for " + ", ".join(missing))
            rclpy.spin_once(self, timeout_sec=1.0)
        nav.destroy()
        return False

    def run(self):
        retries = self.spec.get("retries", {})
        steps = self.spec["steps"]
        # a clean planning scene: move_group keeps the boxes a previous run
        # left, expressed in base_link from wherever the base stood then
        self.clear_scene()
        for i, step in enumerate(steps, 1):
            verb = step["do"]
            fn = getattr(self, f"do_{verb}", None)
            if fn is None:
                self.say(f"{i}/{len(steps)} unknown verb '{verb}'")
                return False
            args = ', '.join(f'{a}={v}' for a, v in step.items() if a != 'do')
            self.say(f"{i}/{len(steps)} {verb} {args}")
            self._prefix = f"{i}/{len(steps)} "
            attempt, result = 0, RETRY
            while result == RETRY and attempt <= retries.get(verb, 0):
                if attempt:
                    self.say(f"   retry {attempt}/{retries.get(verb, 0)}")
                    self._prefix = f"{i}/{len(steps)} retry {attempt}: "
                result = fn(step, attempt) if verb != "posture" else fn(step)
                attempt += 1
            self.log.append((i, verb, result, attempt))
            if result != OK:
                self.say(f"step {i} ({verb}) {result} after {attempt} attempt(s); stopping")
                self._prefix = ""
                self.show(f"step {i} {verb} {result}: stopped")
                self.report()
                return False
        self.say("done")
        self._prefix = ""
        self.show("done")
        self.report()
        return True

    def report(self):
        for i, verb, result, attempts in self.log:
            plural = 's' if attempts != 1 else ''
            self.say(f"   {i:2d} {verb:8s} {result:6s} ({attempts} attempt{plural})")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        if not node.wait_for_stack():
            node.say("mission: the stack did not come up; not starting")
        elif node.nav.pose(timeout=30.0) is None:
            node.say("mission: no map->base_link transform; is localization up?")
        else:
            node.check_heading()
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
