"""Room travel and docking for the mission.

Nav2 (NavigateToPose, MPPI on the lidar costmaps) is for anything further than
SHORT_HOP; the moves between stations in the kitchen's lane are shorter, and go
by hop_to: turn towards a point, drive straight to it, turn to a heading. The
last turn's pivot is 8 cm behind base_link, so the straight leg ends where that
turn will carry the base onto the point (turn_shift).

A dock is made with approach: forwards onto a goal pose along a curve that
takes out lateral and heading error on the way -- terminal guidance, bounded
curvature -- driven on odometry (the goal put into the odom frame), because
AMCL moves its estimate by a centimetre or two every 5 cm of travel and a
goal held in the map moves with it. The mission (mission_node.come_in) decides
between backing straight out for a run-up, driving straight along the dock's
line when the base is already on it, and a hop to a run-up point first.

Nothing here turns on the spot beside a counter by more than the last degree
or two of an approach: a skid-steer turning in place scrubs all four wheels
and goes where the loaded pair drags it.

The pose is the map->base_link transform (AMCL over the EKF), or odom->base_link
where asked for.
"""

import math
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


CREEP_SPEED = 0.12
MAX_TURN = 0.8
# closer than this, do not bother Nav2 (see navigate_to): the station-to-station
# moves along the counter are 0.4-0.95 m in a lane the map knows is free, and
# MPPI dithered for 90 s on one of them
# (1.5: the 0.92 m move from the door to the jam station tipped over 1.0
# by a centimetre of localization and went to Nav2, which aborted it)
SHORT_HOP = 1.5
LIDAR_X = 0.3485          # lidar_link ahead of base_link
BUMPER_X = 0.4454         # the front of the chassis ahead of base_link
# A creep stops this close to whatever is ahead. The SICK's 0.12 m blind
# radius is the device's and did not shrink with the chassis: it used to
# end 0.6 cm past the bumper, and with the lidar 6 cm further back it now
# ends 2.3 cm past it. At 0.03 that left 7 mm before the counter vanished
# into the blind zone and the creep drove on, so the stop is 0.045; the
# 7.9 cm the shorter chassis gained pays for it many times over. (At 7 cm
# the nose stopped short of the plinth and the honey jar was out of reach.)
CLEARANCE_STOP = 0.045
# The approach (see approach): the base steers onto the goal's line along the
# cubic that reaches it with no lateral error and the goal's heading, re-planned
# every step from where it is, and never tighter than KAPPA_MAX: a skid-steer
# on a curve much under 0.4 m radius scrubs like a spin, and the odometry
# stops seeing where it goes (measured: 33 cm travelled on a curve it logged
# as 21). The run-up this needs is RUNUP_PER_LATERAL metres per metre of
# sideways error (simulated: 8 cm in 0.35 m ends within 3 mm and 4 deg).
K_RHO = 1.4                # speed per metre still to go
# Where a turn on the spot pivots, in base_link: 8.4 +- 0.8 cm behind it and
# 1 +- 1.5 cm to the right, over seventeen turns of 70-118 deg in two errands,
# arms tucked or a jar in the lightly pinched hand alike (the four wheels
# scrub, and the pair that carries more of the column's weight holds). A
# 90 deg turn therefore carries base_link 12 cm; hop_to allows for it.
PIVOT = (-0.084, -0.010)
# ...but a rotation of a few tens of degrees, on the spot or along an
# approach's curve, slides the base as if about a point nearly twice as far
# back: 7.6-8.9 cm sideways for 29-33 deg, whichever way it was made
# (measured against the truth). The odometry sees none of it; approach adds
# it back (see there). Fitted to the curve it is used for, into the
# microwave's cavity through 25-26 deg: (+1.5, -6.6) cm, +-0.4, over five
# errands.
CURVE_PIVOT = (-0.139, -0.066)
APPROACH_SPEED = 0.22      # m/s, the most it asks for
APPROACH_TOL_YAW = math.radians(1.5)   # heading error the approach turns out at the end
KAPPA_MAX = 2.5            # 1/m
RUNUP_PER_LATERAL = 4.5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def steer(along, lateral, psi):
    """approach's guidance law: (speed, curvature) from the goal-frame error
    (along < 0 before the goal, lateral > 0 left of its line, psi the
    heading off it). The cubic from here that arrives on the line with the
    goal's heading, and the curvature it starts with, bounded."""
    d = max(-along, 0.10)
    kappa = (-6.0 * lateral / d ** 2 - 4.0 * math.tan(psi) / d) * math.cos(psi) ** 3
    kappa = max(-KAPPA_MAX, min(KAPPA_MAX, kappa))
    # slower as the goal nears, never below a crawl that still breaks
    # static friction
    v = max(0.035, min(APPROACH_SPEED, K_RHO * -along))
    return v, kappa


def approach_path(x, y, th, gx, gy, gyaw, tol=0.005, dt=0.04):
    """What approach does from (x, y, th) to the goal, on the guidance law
    alone (no delays; the slide it aims off for is added at the end):
    (along, lateral, psi) where it stops, and the tightest curvature it
    asked for on the way. For choosing where an approach can start from
    (mission.face_for)."""
    th0 = th
    sx, sy = turn_shift(th0, wrap(gyaw - th0), CURVE_PIVOT)
    ox, oy = gx - sx, gy - sy               # where the odometry is sent
    c, s = math.cos(gyaw), math.sin(gyaw)
    tightest = 0.0
    for _ in range(2000):
        along = c * (x - ox) + s * (y - oy)
        lateral = -s * (x - ox) + c * (y - oy)
        psi = wrap(th - gyaw)
        if along >= -tol or abs(psi) > 1.2:
            break
        v, kappa = steer(along, lateral, psi)
        tightest = max(tightest, abs(kappa))
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v * kappa * dt
    sx, sy = turn_shift(th0, wrap(th - th0), CURVE_PIVOT)
    x, y = x + sx, y + sy
    psi = wrap(th - gyaw)
    if APPROACH_TOL_YAW < abs(psi) < 0.5:       # approach's last turn on the spot
        sx, sy = turn_shift(th, -psi)
        x, y, th = x + sx, y + sy, gyaw
    return c * (x - gx) + s * (y - gy), -s * (x - gx) + c * (y - gy), wrap(th - gyaw), tightest


def turn_shift(heading, turn, pivot=PIVOT):
    """How far (map) base_link moves when the base turns on the spot by
    `turn` from `heading`, about `pivot`: (I - R(turn)) p, in the start
    frame. Shifts add up: a turn made in two parts moves it as far as the
    whole turn does."""
    px, py = pivot
    c, s = math.cos(turn), math.sin(turn)
    dx, dy = (1 - c) * px + s * py, -s * px + (1 - c) * py
    ch, sh = math.cos(heading), math.sin(heading)
    return ch * dx - sh * dy, sh * dx + ch * dy


class Navigator:
    def __init__(self, node, cmd_topic="/base_controller/cmd_vel_unstamped"):
        self.node = node
        self.log = node.get_logger()
        self._cmd = node.create_publisher(Twist, cmd_topic, 10)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, node)
        self._nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._clear = {
            name: node.create_client(ClearEntireCostmap,
                                     f"/{name}_costmap/clear_entirely_{name}_costmap")
            for name in ("local", "global")}
        # the lidar, for the one thing the creep must never do: push the
        # nose into a counter (it did, once: the wheels spun, odometry
        # ran on, and AMCL ended 0.6 m inside the cabinet)
        self._scan = None
        # the turns' pivot shifts (turn_shift) since reset_slip, in odom: the
        # odometry cannot see them, dead_reckoning adds them back
        self.slip = [0.0, 0.0]
        self.on_tick = None
        self._last_tick = 0.0
        node.create_subscription(LaserScan, "/scan_filtered", self._on_scan,
                                 qos_profile_sensor_data)

    def _on_scan(self, msg):
        self._scan = msg

    def _spin_once(self):
        """One turn of the executor inside a driving loop, and every quarter
        second the `on_tick` hook -- the mission keeps the head's gaze on
        what it is driving to with it."""
        rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.on_tick is not None:
            now = time.time()
            if now - self._last_tick >= 0.25:
                self._last_tick = now
                self.on_tick()

    def front_clearance(self, half_width=0.306):
        """Metres between the bumper and the nearest lidar return straight
        ahead (a strip the chassis' width wide); large when nothing is
        seen or the scan has not arrived yet."""
        if self._scan is None:
            return 9.0
        best = 9.0
        a = self._scan.angle_min
        for r in self._scan.ranges:
            if self._scan.range_min < r < self._scan.range_max and abs(a) < math.radians(60):
                x, y = r * math.cos(a), r * math.sin(a)
                if abs(y) < half_width and x > 0:
                    best = min(best, x + LIDAR_X - BUMPER_X)
            a += self._scan.angle_increment
        return best

    def nose_on_something(self):
        """Is the bumper as close to whatever is ahead as a creep would go?"""
        return self.front_clearance() < CLEARANCE_STOP + 0.01

    # ------------------------------------------------------------ state
    def pose(self, timeout=2.0, frame="map"):
        """(x, y, yaw) of base_link in `frame` (map, or odom), or None if TF
        is not up."""
        end = time.time() + timeout
        while rclpy.ok():
            try:
                tf = self._tf.lookup_transform(frame, "base_link",
                                               rclpy.time.Time())
                t = tf.transform.translation
                return t.x, t.y, yaw_of(tf.transform.rotation)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if time.time() > end:
                    return None
                rclpy.spin_once(self.node, timeout_sec=0.1)
        return None

    def odom_in_map(self):
        """(x, y, yaw) of the odom frame in the map -- AMCL's current
        correction -- or None without TF."""
        try:
            tf = self._tf.lookup_transform("map", "odom", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def map_to_odom(self, x, y, yaw, anchor=None):
        """A planar map pose in the odom frame, through `anchor` (odom's pose
        in the map, see odom_in_map) or, without one, AMCL's current
        correction. The odometry frame never jumps: a goal held in it stays
        put relative to the robot while AMCL re-estimates, which a map goal
        does not -- AMCL moves its estimate by a centimetre or two every
        5 cm of travel, and a goal a detection put there moves with it (see
        approach)."""
        anchor = anchor or self.odom_in_map()
        if anchor is None:
            return None
        tx, ty, r = anchor
        c, s = math.cos(r), math.sin(r)
        dx, dy = x - tx, y - ty
        return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - r)

    def reset_slip(self):
        self.slip = [0.0, 0.0]

    def dead_reckoning(self, anchor):
        """base_link in the map through `anchor` (odom's pose in the map when
        it was last right) and the odometry since, with the turns' pivot
        shifts added back (slip): where the base is when AMCL cannot be
        believed -- by the dining table, 15 cm out after a hop, and the scan
        match that saw it refused for a poor fit (measured)."""
        ox, oy, oth = self.pose(frame="odom")
        ox, oy = ox + self.slip[0], oy + self.slip[1]
        tx, ty, r = anchor
        c, s = math.cos(r), math.sin(r)
        return tx + c * ox - s * oy, ty + s * ox + c * oy, wrap(oth + r)

    def odom_target(self, x, y, yaw, anchor):
        """The odometry pose that dead-reckons to the map pose (x, y, yaw):
        map_to_odom through `anchor`, less the slip the odometry has not
        seen."""
        gx, gy, gyaw = self.map_to_odom(x, y, yaw, anchor)
        return gx - self.slip[0], gy - self.slip[1], gyaw

    def spin_for(self, seconds):
        end = self.node.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.node.get_clock().now() < end:
            self._spin_once()

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self._cmd.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def settle(self, seconds=0.8):
        """Stop and let the base come to rest before trusting where it is.

        The wheels keep rolling for a moment after the last command, and
        0.05 m of coast plus 10 degrees of yaw moves the shoulder enough to
        turn a 0.693 m reach into a 0.733 m one -- past the arm's limit.
        """
        self.stop()
        self.spin_for(seconds)

    # ------------------------------------------------------------ Nav2
    def clear_costmaps(self):
        for name, client in self._clear.items():
            if client.wait_for_service(timeout_sec=2.0):
                fut = client.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)

    def navigate_to(self, x, y, yaw, timeout=90.0, before_turn=None):
        """Get to a pose: Nav2 for anything further than a short hop.

        A goal within SHORT_HOP of the base is driven with the local
        turn-drive-turn primitive instead. MPPI with this footprint handles
        room travel well and a nearby goal badly: asked to finish 0.4 m
        beside or behind itself it dithers, the progress checker aborts it,
        and while it dithers it can shove the base into furniture -- and a
        base pushing against a chair spins its wheels, which the odometry
        reports as motion and AMCL then believes (measured: 1 m of error from
        one such episode). Short hops only ever happen between staging poses
        in the lane, along floor the base has just driven.

        The Nav2 deadline is in SIM time: the robot moves on the simulation
        clock, and that clock need not run in real time (RainBot measured
        0.57x under load). A wall-clock backstop only guards against a
        stopped /clock.
        """
        here = self.pose()
        if here is not None and math.hypot(x - here[0], y - here[1]) < SHORT_HOP:
            return self.hop_to(x, y, yaw, before_turn=before_turn)
        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.log.error("nav: NavigateToPose server not available")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.log.info(f"nav: -> ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")
        send = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.log.warn("nav: goal rejected")
            return False
        result = handle.get_result_async()
        deadline = self.node.get_clock().now() + Duration(seconds=timeout)
        wall_stop = time.time() + timeout * 4.0
        while rclpy.ok() and not result.done():
            rclpy.spin_until_future_complete(self.node, result, timeout_sec=0.5)
            if self.node.get_clock().now() >= deadline or time.time() > wall_stop:
                self.log.warn(f"nav: {timeout:.0f} s elapsed, cancelling")
                handle.cancel_goal_async()
                self.spin_for(1.0)
                self.stop()
                return False
        status = result.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.settle(0.5)
            px, py, pyaw = self.pose()
            self.log.info(f"nav: arrived at ({px:.2f}, {py:.2f}, "
                          f"{math.degrees(pyaw):.0f} deg)")
            return True
        self.log.warn(f"nav: goal ended with status {status}")
        self.stop()
        return False

    def approach(self, gx, gy, gyaw, tol=0.005, tol_yaw=APPROACH_TOL_YAW,
                 stop_clear=CLEARANCE_STOP, timeout=30.0, frame="map"):
        """Drive FORWARDS onto the pose (gx, gy, gyaw) along a curve that
        takes out lateral and heading error on the way, instead of spinning
        on the spot to fix them afterwards; the last degree or two of heading
        is then turned in place. Stops when the base draws level with the
        goal along its line, short of it if the bumper comes within
        stop_clear of anything.

        The steering is terminal guidance: at every step, the cubic from here
        to the goal that arrives on its line with its heading, and the
        curvature that cubic starts with -- kappa = (-6 e / d^2 - 4 tan(psi) / d)
        cos^3(psi), for lateral error e, heading error psi and d still to go
        -- held under KAPPA_MAX. The gains grow as the goal nears, which is
        what brings e and psi to zero together; d is floored so they stay
        finite. A skid-steer spinning on the spot is dragged by whichever
        wheels carry more load, while on a gentle curve all four roll.

        frame="odom" takes the goal in the odometry frame (map_to_odom): the
        approach then drives on odometry alone, and AMCL's corrections on
        the way -- 1-5 cm while moving, gone once it stops (measured against
        the simulator's truth) -- cannot drag it about. What the odometry
        cannot see is the sideways slide that comes with turning: 7.7 cm on
        a curve through 29 deg (measured). It depends only on how far the
        heading turns, which the goal fixes, so the odometry is sent that
        much short of the goal (CURVE_PIVOT) and the slide it then makes
        goes into `slip` for the dead reckoning after. (Steering against it
        as it came instead ended 10 deg off the goal's heading, simulated.)
        Returns the (x, y, yaw) it ended at, in `frame`, slide included."""
        th0 = self.pose(frame=frame)[2]
        aim = turn_shift(th0, wrap(gyaw - th0), CURVE_PIVOT)
        gx, gy = gx - aim[0], gy - aim[1]
        c, s = math.cos(gyaw), math.sin(gyaw)
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        why = "timed out"
        while rclpy.ok() and self.node.get_clock().now() < end:
            x, y, th = self.pose(frame=frame)
            along = c * (x - gx) + s * (y - gy)          # < 0 before the goal
            lateral = -s * (x - gx) + c * (y - gy)       # > 0 left of its line
            psi = wrap(th - gyaw)
            if along >= -tol:
                why = "there"
                break
            if abs(psi) > 1.2:
                why = f"heading {math.degrees(psi):+.0f} deg off the line"
                break
            if self.front_clearance() < stop_clear:
                why = f"{self.front_clearance():.3f} m to whatever is ahead"
                break
            v, kappa = steer(along, lateral, psi)
            self.drive(v, v * kappa)
            self._spin_once()
        self.settle(0.3)
        x, y, th = self.pose(frame=frame)
        # the slide the curve made, which the odometry was sent short of
        sx, sy = turn_shift(th0, wrap(th - th0), CURVE_PIVOT)
        if frame == "odom":
            self.slip[0] += sx
            self.slip[1] += sy
        if abs(wrap(gyaw - th)) > tol_yaw and abs(wrap(gyaw - th)) < 0.5:
            before = th
            self.turn_to(gyaw, tol=tol_yaw, frame=frame)     # books its own
            x, y, th = self.pose(frame=frame)
            tx, ty = turn_shift(before, wrap(th - before))
            sx, sy = sx + tx, sy + ty
        gx, gy = gx + aim[0], gy + aim[1]
        x, y = x + sx, y + sy
        along = c * (x - gx) + s * (y - gy)
        lateral = -s * (x - gx) + c * (y - gy)
        self.log.info(f"approach: {why}; {along * 100:+.1f} cm along / {lateral * 100:+.1f} cm "
                      f"across the goal, heading {math.degrees(wrap(th - gyaw)):+.1f} deg "
                      f"({frame})")
        return x, y, th

    def hop_to(self, x, y, yaw, tol=0.03, timeout=30.0, before_turn=None):
        """Turn towards (x, y), drive straight to it, face yaw (all map). Local
        and blind: for short moves over floor known to be free.

        Planned and driven on odometry, with both turns allowed for: a turn
        on the spot carries base_link about PIVOT (turn_shift), which neither
        the wheels nor, for a second or two, AMCL can see -- an 83 deg turn
        moved the base 10 cm that AMCL never caught, the straight leg was
        then driven from the wrong place, and the hop ended 14 cm off
        (measured). The odometry does see the straight leg exactly. So the
        leg is aimed so that the base, shifted by the first turn before it
        and by the last one after it, stops at (x, y). `before_turn`, when
        given, is called between the leg and the last turn (the mission
        relocalizes there when the run-up point itself cannot be matched)."""
        goal = self.map_to_odom(x, y, yaw)
        frame = "odom"
        if goal is None:
            goal, frame = (x, y, yaw), "map"
        gx, gy, gyaw = goal
        ox, oy, oth = self.pose(frame=frame)
        dx, dy = gx, gy
        for _ in range(3):
            leg = math.atan2(dy - oy, dx - ox)
            s1 = turn_shift(oth, wrap(leg - oth))
            s2 = turn_shift(leg, wrap(gyaw - leg))
            dx, dy = gx - s1[0] - s2[0], gy - s1[1] - s2[1]
        dist = math.hypot(dx - ox, dy - oy)
        self.log.info(f"nav: short hop {dist:.2f} m for ({x:.2f}, {y:.2f}, "
                      f"{math.degrees(yaw):+.0f} deg)")
        if dist > tol:
            if not self.turn_to(math.atan2(dy - oy, dx - ox), frame=frame):
                return False
            end = self.node.get_clock().now() + Duration(seconds=timeout)
            while rclpy.ok() and self.node.get_clock().now() < end:
                cx, cy, ch = self.pose(frame=frame)
                left = math.hypot(dx - cx, dy - cy)
                if left < tol:
                    break
                err = wrap(math.atan2(dy - cy, dx - cx) - ch)
                if abs(err) > 1.2:          # overshot: stop, do not circle
                    break
                # slowing to a crawl for the last decimetre: at 0.08 m/s
                # the base coasted 6-10 cm past the goal (measured, once
                # the odometry stopped under-driving it)
                speed = max(0.05, min(0.25, 1.5 * left))
                self.drive(speed, max(-0.5, min(0.5, 1.5 * err)))
                self._spin_once()
            self.settle(0.4)
        if before_turn is not None:
            before_turn()
        ok = self.turn_to(gyaw, frame=frame)
        cx, cy, ch = self.pose()
        self.log.info(f"nav: hop landed at ({cx:.2f}, {cy:.2f}, {math.degrees(ch):+.0f} deg), "
                      f"{math.hypot(x - cx, y - cy):.2f} m from the goal")
        return ok

    # ------------------------------------------------------------ docking
    def turn_to(self, heading, tol=0.03, timeout=25.0, frame="map"):
        """Spin on the spot to a heading. Only safe where the 0.6 m turning
        circle is clear -- at a staging pose, not beside a counter. The
        turn's pivot shift (turn_shift) goes into `slip`."""
        start = self.pose(frame="odom")[2]
        try:
            return self._turn_to(heading, tol, timeout, frame)
        finally:
            turned = wrap(self.pose(frame="odom")[2] - start)
            sx, sy = turn_shift(start, turned)
            self.slip[0] += sx
            self.slip[1] += sy

    def _turn_to(self, heading, tol, timeout, frame):
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.node.get_clock().now() < end:
            err = wrap(heading - self.pose(frame=frame)[2])
            if abs(err) < tol:
                self.settle(0.3)
                return True
            # Floored: a skid-steer needs a real shove to break static friction.
            rate = max(0.25, min(MAX_TURN, 2.0 * abs(err)))
            self.drive(0.0, rate if err > 0 else -rate)
            self._spin_once()
        self.stop()
        self.log.warn(f"turn_to gave up: wanted {math.degrees(heading):+.1f} deg, "
                      f"at {math.degrees(self.pose(frame=frame)[2]):+.1f}")
        return False

    def creep(self, done, limit, speed=CREEP_SPEED, timeout=30.0):
        """Drive straight ahead until done() is true or `limit` metres pass.

        Straight only: the base cannot turn where it works. Heading is held
        by a small correction, because 13 degrees of yaw swings the shoulder
        0.11 m -- the difference between in reach and out of it.
        """
        sx, sy, heading = self.pose()
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.node.get_clock().now() < end:
            if done():
                self.settle()
                return done()
            x, y, yaw = self.pose()
            if math.hypot(x - sx, y - sy) > limit:
                break
            if self.front_clearance() < CLEARANCE_STOP:
                self.log.warn(f"creep: {self.front_clearance():.2f} m to whatever is ahead; "
                              "stopping")
                break
            self.drive(speed, max(-0.3, min(0.3, 1.5 * wrap(heading - yaw))))
            self._spin_once()
        self.settle()
        return done()

    def back_out(self, distance, speed=0.2, timeout=30.0):
        """Reverse straight along the current heading -- the line it drove in
        on, which is the one stretch of floor known to be free."""
        sx, sy, heading = self.pose()
        end = self.node.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.node.get_clock().now() < end:
            x, y, yaw = self.pose()
            if math.hypot(x - sx, y - sy) >= distance:
                break
            self.drive(-speed, max(-0.3, min(0.3, 1.5 * wrap(heading - yaw))))
            self._spin_once()
        self.settle(0.5)
        return True
