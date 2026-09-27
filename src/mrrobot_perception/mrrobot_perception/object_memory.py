"""Object memory with an active head search.

Keeps the latest map-frame pose of every object the colour detector reports,
with its age, and answers /object_memory/find (mrrobot_msgs/FindObject):

  * fresh enough  -> straight from memory, no motion;
  * otherwise, if the caller says where the object should be (`near`) -> the
    head is aimed there first, one look;
  * otherwise, if the caller allows a sweep -> the head is driven through a
    pan/tilt grid (head_controller's FollowJointTrajectory action, waiting on
    each result, then a short settle for the detector) until the object is
    reported, and the head is left looking at it.

The mission calls find() and never reads the camera. RainBot cannot do this:
its camera is fixed on the wrist and an object out of frame is simply lost.
Positions are in the map frame, so a detection made from one station is
still valid after driving to the next.
"""

import math
import time

import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from mrrobot_msgs.srv import FindObject
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

HEAD_JOINTS = ["openarmx_head_pitch_joint", "openarmx_head_yaw_joint"]
# the head's joint limits (openarmx_head_description): pitch nods down for
# negative, yaw turns left for positive
PITCH_RANGE = (-1.086, 0.403)
YAW_RANGE = (-1.5708, 1.5708)
# the camera sits about this far up and forward of head_base_link (the pitch
# axis 4 cm up, the yaw axis 7 cm above that, the lens 5.5 cm higher and
# 8 cm forward): close enough to aim a 70 x 87 degree field of view
CAMERA_UP, CAMERA_FORWARD = 0.16, 0.08


class ObjectMemory(Node):
    def __init__(self):
        super().__init__("object_memory")
        self.declare_parameter("fresh_seconds", 30.0)
        self.declare_parameter("sweep_pans", [0.0, -0.6, 0.6, -1.2, 1.2])
        self.declare_parameter("sweep_tilts", [-0.35, -0.7])
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("smoothing", 8)
        self._memory = {}        # name -> list of (stamp_ns, (x, y, z))
        self._group = ReentrantCallbackGroup()
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_subscription(Detection3DArray, "/mrRobot/detections",
                                 self._on_detections, 10, callback_group=self._group)
        self._head = ActionClient(self, FollowJointTrajectory,
                                  "/head_controller/follow_joint_trajectory",
                                  callback_group=self._group)
        self.create_service(FindObject, "/object_memory/find", self._find,
                            callback_group=self._group)
        self._pub = self.create_publisher(Detection3DArray, "/mrRobot/objects", 10)
        from visualization_msgs.msg import MarkerArray
        self._markers = self.create_publisher(MarkerArray, "/mrRobot/objects/markers", 10)
        self.create_timer(1.0, self._publish_memory, callback_group=self._group)
        self.get_logger().info("object memory ready; /object_memory/find")

    # ------------------------------------------------------------ memory
    def _on_detections(self, msg):
        now = self.get_clock().now().nanoseconds
        for det in msg.detections:
            p = det.bbox.center.position
            hist = self._memory.setdefault(det.id, [])
            hist.append((now, (p.x, p.y, p.z)))
            del hist[:-self.get_parameter("smoothing").value]

    def _estimate(self, name, window=1.5, near=None):
        """(age_s, (x, y, z)) from the median of the detections in the last
        `window` seconds -- and only if there are at least two of them that
        agree within 5 cm: one frame of something jar-coloured at jar size
        (a slice of the cereal box, once) is not an object. With `near`
        ((x, y), radius) only detections inside that circle count: the
        kitchen has two jam jars, and the errand wants the one by the
        microwave."""
        hist = self._memory.get(name)
        if not hist:
            return None
        now = self.get_clock().now().nanoseconds
        recent = [h for h in hist if now - h[0] < window * 1e9]
        if near is not None:
            recent = [h for h in recent if math.hypot(h[1][0] - near[0][0], h[1][1] - near[0][1]) < near[1]]
        if len(recent) < 2:
            return None
        xs, ys, zs = (sorted(h[1][i] for h in recent) for i in range(3))
        mid = len(xs) // 2
        med = (xs[mid], ys[mid], zs[mid])
        agreeing = [h for h in recent if math.dist(h[1], med) < 0.05]
        if len(agreeing) < 2:
            return None
        return (now - agreeing[-1][0]) * 1e-9, med

    def _publish_memory(self):
        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        from vision_msgs.msg import Detection3D
        from visualization_msgs.msg import Marker, MarkerArray
        markers = MarkerArray()
        for name in self._memory:
            est = self._estimate(name)
            if est is None:
                continue
            det = Detection3D()
            det.header = out.header
            det.id = name
            det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = est[1]
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = det.bbox.size.y = 0.09
            det.bbox.size.z = 0.1
            out.detections.append(det)
            for kind, mid in ((Marker.CYLINDER, 0), (Marker.TEXT_VIEW_FACING, 1)):
                m = Marker()
                m.header = out.header
                m.ns, m.id, m.type, m.action = name, mid, kind, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = est[1]
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = 0.09
                m.scale.z = 0.115 if kind == Marker.CYLINDER else 0.08
                m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.6, 0.0, 0.8) if name == "honey" else (0.8, 0.0, 0.1, 0.8)
                if kind == Marker.TEXT_VIEW_FACING:
                    m.text = f"{name} ({est[0]:.0f} s)"
                    m.pose.position.z += 0.12
                    m.color.r = m.color.g = m.color.b = 1.0
                m.lifetime.sec = 3
                markers.markers.append(m)
        self._pub.publish(out)
        self._markers.publish(markers)

    # ------------------------------------------------------------ search
    def _aim(self, point):
        """(pitch, yaw) that points the camera at a map point, or None
        without TF. head_base_link faces +y; yaw turns that towards -x,
        pitch nods it towards -z."""
        try:
            t = self._tf.lookup_transform("head_base_link", "map", rclpy.time.Time(),
                                          timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"aim: no transform for head_base_link: {e}")
            return None
        q, tr = t.transform.rotation, t.transform.translation
        # rotate then translate: p_head = R * p_map + t
        x, y, z = point
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # quaternion rotation of a vector, expanded
        ix, iy, iz, iw = (qw * x + qy * z - qz * y, qw * y + qz * x - qx * z,
                          qw * z + qx * y - qy * x, -qx * x - qy * y - qz * z)
        px = ix * qw + iw * -qx + iy * -qz - iz * -qy + tr.x
        py = iy * qw + iw * -qy + iz * -qx - ix * -qz + tr.y
        pz = iz * qw + iw * -qz + ix * -qy - iy * -qx + tr.z
        yaw = math.atan2(-px, py)
        pitch = math.atan2(pz - CAMERA_UP, math.hypot(px, py) - CAMERA_FORWARD)
        return (max(PITCH_RANGE[0], min(PITCH_RANGE[1], pitch)),
                max(YAW_RANGE[0], min(YAW_RANGE[1], yaw)))

    def _look(self, pitch, yaw, seconds=None):
        if not self._head.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("no head controller action server")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = HEAD_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(pitch), float(yaw)]
        if seconds is None:
            # the head is light and quick: a hop of a radian in under a second
            now = self._head_now()
            far = 0.0 if now is None else max(abs(pitch - now[0]), abs(yaw - now[1]))
            seconds = max(0.4, 0.3 + far / 1.5)
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        handle = self._head.send_goal_async(goal)
        while not handle.done():
            time.sleep(0.02)
        if not handle.result().accepted:
            return False
        result = handle.result().get_result_async()
        while not result.done():
            time.sleep(0.02)
        return result.result().result.error_code == 0

    def _head_now(self):
        """The head's (pitch, yaw) from TF, or None."""
        try:
            p = self._tf.lookup_transform("head_base_link", "head_pitch_link", rclpy.time.Time())
            y = self._tf.lookup_transform("head_pitch_link", "head_yaw_link", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        pq, yq = p.transform.rotation, y.transform.rotation
        pitch = math.atan2(2.0 * (pq.w * pq.x + pq.y * pq.z),
                           1.0 - 2.0 * (pq.x * pq.x + pq.y * pq.y))
        yaw = math.atan2(2.0 * (yq.w * yq.z + yq.x * yq.y),
                         1.0 - 2.0 * (yq.y * yq.y + yq.z * yq.z))
        return pitch, yaw

    def _settle_and_estimate(self, name, near):
        settle = self.get_parameter("settle_seconds").value
        time.sleep(settle)
        return self._estimate(name, window=settle, near=near)

    def _find(self, request, response):
        name = request.colour
        max_age = request.max_age if request.max_age > 0 else self.get_parameter("fresh_seconds").value
        near = None
        if request.near_radius > 0:
            near = ((request.near.x, request.near.y), request.near_radius)
        est = self._estimate(name, window=max(max_age, 1.5), near=near)
        if est is not None and est[0] <= max_age:
            return self._answer(response, name, est, "from memory")
        if not request.sweep:
            response.found = False
            response.message = f"{name}: nothing fresh in memory and no sweep allowed"
            return response
        seen_before = None if est is None else est[0]
        # where the caller expects it first: one look instead of a sweep
        aim = None if near is None else self._aim((near[0][0], near[0][1], request.near.z))
        if aim is not None:
            self.get_logger().info(f"find {name}: looking where it should be, "
                                   f"pitch {aim[0]:+.2f} yaw {aim[1]:+.2f}")
            if self._look(*aim):
                est = self._settle_and_estimate(name, near)
                if est is not None:
                    return self._answer(response, name, est, "seen where it should be")
        for tilt in self.get_parameter("sweep_tilts").value:
            for pan in self.get_parameter("sweep_pans").value:
                self.get_logger().info(f"find {name}: looking pitch {tilt:+.2f} yaw {pan:+.2f}")
                if not self._look(tilt, pan):
                    continue
                est = self._settle_and_estimate(name, near)
                if est is not None:
                    return self._answer(response, name, est,
                                        f"seen after sweeping to pitch {tilt:+.2f} yaw {pan:+.2f}")
        response.found = False
        response.message = (f"{name}: not seen in a full sweep"
                            + ("" if seen_before is None else f"; last seen {seen_before:.0f} s ago"))
        self.get_logger().warn(response.message)
        return response

    def _answer(self, response, name, est, how):
        age, (x, y, z) = est
        response.found = True
        response.age = float(age)
        response.pose = PoseStamped()
        response.pose.header.frame_id = "map"
        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.pose.position.x, response.pose.pose.position.y, response.pose.pose.position.z = x, y, z
        response.pose.pose.orientation.w = 1.0
        response.message = f"{name} at ({x:.3f}, {y:.3f}, {z:.3f}), {age:.1f} s old, {how}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMemory()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
