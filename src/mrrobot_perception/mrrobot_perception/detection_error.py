"""Detection error against Webots ground truth.

Compares every /mrRobot/detections entry with the true pose of the object it
names (/ground_truth/<name>, from the supervisor plugin) and logs the error,
horizontal and vertical separately: the detector reports the jar's axis at
the visible body's mid-height, the supervisor reports the jar's base, so the
vertical offset is expected and reported as such.

    ros2 run mrrobot_perception detection_error
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray


class DetectionError(Node):
    def __init__(self):
        super().__init__("detection_error")
        self.declare_parameter("objects", ["honey", "jam"])
        self.declare_parameter("log_every", 2.0)
        self._truth = {}
        self._stats = {}
        for name in self.get_parameter("objects").value:
            self.create_subscription(PoseStamped, f"/ground_truth/{name}",
                                     (lambda n: lambda m: self._truth.__setitem__(n, m))(name), 10)
        self.create_subscription(Detection3DArray, "/mrRobot/detections", self._on_det, 10)
        self._last_log = 0.0

    def _on_det(self, msg):
        for det in msg.detections:
            t = self._truth.get(det.id)
            if t is None:
                continue
            p, q = det.bbox.center.position, t.pose.position
            horiz = math.hypot(p.x - q.x, p.y - q.y)
            vert = p.z - q.z
            s = self._stats.setdefault(det.id, [])
            s.append(horiz)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_log >= self.get_parameter("log_every").value:
                self._last_log = now
                self.get_logger().info(
                    f"{det.id}: horizontal error {horiz * 100:.1f} cm "
                    f"(mean {sum(s) / len(s) * 100:.1f}, max {max(s) * 100:.1f}, n={len(s)}); "
                    f"detected {vert * 100:+.1f} cm above the jar's base")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
