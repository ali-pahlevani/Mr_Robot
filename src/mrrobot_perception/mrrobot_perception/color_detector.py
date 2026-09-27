"""HSV colour detector on the head RGB-D: objects -> 3D poses in the map frame.

RainBot's approach (threshold in HSV, take the biggest blob), with what its
fixed wrist camera never needed: a real depth image instead of a known-plane
assumption, a physical size gate so a 9 cm jar is not confused with a 30 cm
box of the same colour, and an optional lid-colour check for objects that
only differ by their lid.

Publishes vision_msgs/Detection3DArray on /mrRobot/detections -- one
Detection3D per object seen, `id` = the object name, bbox.center = the
object's axis at the visible body's mid-height, in `target_frame` -- and a
debug image on /mrRobot/detections/image.

Frames: the Webots camera looks along +x of head_camera_link with +y left and
+z up; the image's pixel (u right, v down) deprojects to
(z_depth, -x_right, -y_down) in that frame. The depth image is planar depth
(the z of the OpenGL depth buffer), not ray length.
"""

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point  # noqa: F401  (registers PointStamped)
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import cv2


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.declare_parameter("image_topic", "/head_camera/image_color")
        self.declare_parameter("depth_topic", "/head_depth/image")
        self.declare_parameter("info_topic", "/head_camera/camera_info")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("min_pixels", 40)
        self.declare_parameter("objects", ["honey", "jam"])
        self.objects = {}
        for name in self.get_parameter("objects").value:
            self.declare_parameter(f"{name}.body_hsv", [0, 0, 0, 180, 255, 255])
            self.declare_parameter(f"{name}.size", [0.09, 0.1])
            self.declare_parameter(f"{name}.size_tolerance", 0.5)
            self.declare_parameter(f"{name}.radius", 0.045)
            self.declare_parameter(f"{name}.lid_hsv", [0])
            self.declare_parameter(f"{name}.lid_min_fraction", 0.25)
            # how far the object's reference point (a jar's axis at
            # mid-height) lies BELOW the coloured blob: for a jar whose
            # only colour is its lid, the lid is what gets detected
            self.declare_parameter(f"{name}.below", 0.0)
            # colour that must fill the strip just UNDER the blob (a jar
            # found by its lid has its label there; a logo on a box has
            # more box)
            self.declare_parameter(f"{name}.under_hsv", [0])
            self.declare_parameter(f"{name}.under_min_fraction", 0.4)
            # where the reference point may be, in the target frame's z:
            # a jar stands on a worktop, a logo on a cereal box does not
            self.declare_parameter(f"{name}.height_range", [-10.0, 10.0])
            # how much of the blob's bounding box the colour fills: a
            # checked lid is half white, a printed logo is solid
            self.declare_parameter(f"{name}.fill_range", [0.0, 1.0])
            # the blob's height on its own, when it varies with the view
            # more than the width does (a lid seen from the side or from
            # above); the default is the old rule from size and tolerance
            self.declare_parameter(f"{name}.height_span", [0.0, 0.0])
            p = lambda k: self.get_parameter(f"{name}.{k}").value  # noqa: E731
            lid = list(p("lid_hsv"))
            self.objects[name] = {
                "body": _ranges(p("body_hsv")),
                "size": tuple(p("size")),
                "tol": float(p("size_tolerance")),
                "radius": float(p("radius")),
                "lid": _ranges(lid) if len(lid) >= 6 else None,
                "lid_frac": float(p("lid_min_fraction")),
                "below": float(p("below")),
                "under": _ranges(list(p("under_hsv"))) if len(list(p("under_hsv"))) >= 6 else None,
                "under_frac": float(p("under_min_fraction")),
                "height": tuple(p("height_range")),
                "fill": tuple(p("fill_range")),
                "height_span": tuple(p("height_span")),
            }
        self.bridge = CvBridge()
        self.depth = None
        self.info = None
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.pub = self.create_publisher(Detection3DArray, "/mrRobot/detections", 10)
        self.pub_image = self.create_publisher(Image, "/mrRobot/detections/image", 2)
        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, qos_profile_sensor_data)
        self._pending = None
        # Images are processed from a timer, a beat after they arrive: the
        # transforms for an image stamped "now" are still in flight when its
        # callback runs, and a single-threaded node cannot wait for them
        # inside that callback.
        self.create_timer(1.0 / self.get_parameter("rate").value, self._process)
        self.get_logger().info(f"detecting {sorted(self.objects)} on "
                               f"{self.get_parameter('image_topic').value}")

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_image(self, msg):
        # with the depth frame of the same moment, which may have moved on by
        # the time the colour frame is processed
        self._pending = (msg, self.depth)

    def _process(self):
        item, self._pending = self._pending, None
        if item is None or item[1] is None or self.info is None:
            return
        msg, depth_msg = item
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        if depth.shape != bgr.shape[:2]:
            self.get_logger().warn("depth and colour images differ in size", throttle_duration_sec=10)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        # Two transforms, not one: camera -> odom at the IMAGE's time (the
        # EKF publishes odom continuously, and while the head or base moves
        # the latest transform belongs to a different view -- measured: a
        # jar reported a metre behind the robot during a head sweep), then
        # odom -> map at whatever time AMCL last published it, because AMCL
        # only re-stamps map->odom when the robot moves and a lookup at "now"
        # fails as soon as it stands still.
        target = self.get_parameter("target_frame").value
        try:
            to_odom = self.tf.lookup_transform("odom", msg.header.frame_id, msg.header.stamp)
            to_map = self.tf.lookup_transform(target, "odom", rclpy.time.Time())
        except tf2_ros.ExtrapolationException:
            # The transforms for this frame's instant are one physics step
            # behind it: try the same frame again next time round, unless a
            # newer one has come in. Dropped instead, a frame whose timer
            # tick fell just after its arrival was lost, and when the two
            # stayed in step every frame was -- the head swept the jam jar
            # three times and never reported it (measured).
            if self._pending is None:
                self._pending = item
            return
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"no transform for {msg.header.frame_id}: {e}",
                                   throttle_duration_sec=5)
            return

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = target
        debug = bgr.copy()
        for name, spec in self.objects.items():
            found = self._find(name, spec, hsv, depth, fx, fy, cx, cy, debug)
            if found is None:
                continue
            (x, y, z), (u, v, w, h) = found
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x, pt.point.y, pt.point.z = x, y, z
            pm = do_transform_point(do_transform_point(pt, to_odom), to_map)
            pm.point.z -= spec["below"]
            if not spec["height"][0] <= pm.point.z <= spec["height"][1]:
                cv2.rectangle(debug, (u, v), (u + w, v + h), (255, 0, 255), 1)
                continue
            det = Detection3D()
            det.header = out.header
            det.id = name
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position = pm.point
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center.position = pm.point
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = det.bbox.size.y = 2 * spec["radius"]
            det.bbox.size.z = spec["size"][1]
            out.detections.append(det)
            cv2.rectangle(debug, (u, v), (u + w, v + h), (0, 255, 0), 1)
            cv2.putText(debug, f"{name} {pm.point.x:.2f},{pm.point.y:.2f}", (u, max(v - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        self.pub.publish(out)
        if self.pub_image.get_subscription_count() > 0:
            img = self.bridge.cv2_to_imgmsg(debug, "bgr8")
            img.header = msg.header
            self.pub_image.publish(img)

    def _find(self, name, spec, hsv, depth, fx, fy, cx, cy, debug):
        mask = _mask(hsv, spec["body"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, n):
            u, v, w, h, area = stats[i]
            if area < self.get_parameter("min_pixels").value:
                continue
            blob = labels == i
            d = depth[blob]
            d = d[np.isfinite(d) & (d > 0.05)]
            if d.size < 10:
                continue
            z = float(np.median(d))
            width_m, height_m = w * z / fx, h * z / fy
            ew, eh = spec["size"]
            tol = spec["tol"]
            h_lo, h_hi = spec["height_span"]
            if h_hi <= h_lo:
                h_lo, h_hi = eh * (1 - tol), eh * (1 + tol) * 1.3
            # width, height and their ratio: a slice of a cereal box can be
            # jar-sized in one dimension, rarely in both and in shape
            if not (ew * (1 - tol) <= width_m <= ew * (1 + tol)
                    and h_lo <= height_m <= h_hi
                    and 0.6 * h_lo / ew <= height_m / width_m <= 1.6 * h_hi / ew):
                cv2.rectangle(debug, (u, v), (u + w, v + h), (0, 0, 255), 1)
                if area > 200:
                    cv2.putText(debug, f"{width_m:.2f}x{height_m:.2f}", (u, v + h + 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                continue
            fill = area / float(w * h)
            if not spec["fill"][0] <= fill <= spec["fill"][1]:
                cv2.rectangle(debug, (u, v), (u + w, v + h), (128, 0, 128), 1)
                continue
            if spec["lid"] is not None:
                strip_h = max(2, int(0.035 * fy / z))
                v0 = max(0, v - strip_h)
                strip = hsv[v0:v, u:u + w]
                frac = float(np.mean(_mask(strip, spec["lid"]) > 0)) if strip.size else 0.0
                if frac < spec["lid_frac"]:
                    cv2.rectangle(debug, (u, v), (u + w, v + h), (0, 128, 255), 1)
                    continue
            if spec["under"] is not None:
                strip_h = max(2, int(0.035 * fy / z))
                v1 = min(hsv.shape[0], v + h + strip_h)
                strip = hsv[v + h:v1, u:u + w]
                frac = float(np.mean(_mask(strip, spec["under"]) > 0)) if strip.size else 0.0
                if frac < spec["under_frac"]:
                    cv2.rectangle(debug, (u, v), (u + w, v + h), (255, 128, 0), 1)
                    continue
            if best is None or area > best[0]:
                ys, xs = np.nonzero(blob)
                best = (area, float(xs.mean()), float(ys.mean()), z, (int(u), int(v), int(w), int(h)))
        if best is None:
            return None
        _, uc, vc, z, box = best
        # surface point in the camera's optical sense, then to the Webots
        # (x forward, y left, z up) frame the TF tree carries
        xr, yd = z * (uc - cx) / fx, z * (vc - cy) / fy
        ray = np.array([z, -xr, -yd])
        ray += spec["radius"] * ray / np.linalg.norm(ray)     # surface -> axis
        # `below` is along the world's vertical, which in this camera-fixed
        # frame is only right when the head looks level; the head nods up
        # to 40 deg for a sweep, so it is applied after the transform to
        # the target frame instead (see _process)
        return (float(ray[0]), float(ray[1]), float(ray[2])), box


def _ranges(flat):
    flat = [int(v) for v in flat]
    return [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6])) for i in range(0, len(flat) - 5, 6)]


def _mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
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
