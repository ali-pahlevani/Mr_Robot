"""Ground truth from the Webots supervisor, published for measurement only.

This is a webots_ros2_driver Python plugin, so it runs INSIDE the driver
process with direct access to the simulator: it reads the true pose of the
robot and of every DEF-named object it is given and publishes each as a
PoseStamped in the `map` frame. The map is built and shifted so that map ==
Webots world (see mrrobot_navigation/maps/README.md), which is what makes these
directly comparable with AMCL, the EKF and the colour detector.

Loaded from the URDF:

    <plugin type="mrrobot_perception.ground_truth.GroundTruth">
      <objects>HONEY,JAM</objects>      <!-- optional; this is the default -->
      <rate>10</rate>                   <!-- Hz, optional -->
    </plugin>

The robot's own pose is always published, on /ground_truth/robot. Nothing in
the control path subscribes to any of these topics.

It also offers /ground_truth/snapshot (webots_ros2_msgs/SetString:
"path.png[,bearing_deg[,distance_m]]", bearing relative to the robot's
heading, default 50 and 2.5): it swings the world's DEF VIEWPOINT to look at
the robot from there and exports the 3D view -- the only way to see what the simulator
is actually rendering from a script, and what the mesh checks in the README
were done with.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from webots_ros2_msgs.srv import GetBool, SetString


class GroundTruth:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._step_ms = int(self._robot.getBasicTimeStep())
        rate = float(properties.get("rate", 10.0))
        self._every = max(1, int(round(1000.0 / (rate * self._step_ms))))
        self._count = 0

        # Another Python plugin in the same driver may already have done this.
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("ground_truth")
        self._log = self._node.get_logger()

        self._targets = {}
        if not hasattr(self._robot, "getSelf"):
            self._log.error("ground_truth: robot is not a supervisor -- set "
                            "`supervisor TRUE` on the mrRobot node in the world")
            return
        self._targets["robot"] = self._robot.getSelf()
        names = properties.get("objects", "HONEY,JAM")
        for name in [n.strip() for n in names.split(",") if n.strip()]:
            node = self._robot.getFromDef(name)
            if node is None:
                self._log.warn(f"ground_truth: no DEF {name} in the world")
                continue
            self._targets[name.lower()] = node
        self._pubs = {key: self._node.create_publisher(
            PoseStamped, f"/ground_truth/{key}", 10) for key in self._targets}
        self._node.create_service(SetString, "/ground_truth/snapshot", self._snapshot)
        # Which solids of the robot touch what: every contact point's node
        # (and its parent's) by name. The one question the joint states
        # cannot answer when an arm stops short.
        self._node.create_service(GetBool, "/ground_truth/contacts", self._contacts)
        # The microwave door's hinge angle, dug out of the Oven PROTO
        # (base-node children > HingeJoint > jointParameters.position), on
        # /ground_truth/microwave_door: 0 closed, 1.5 fully open.
        self._door_solid = None
        self._door = self._door_joint()
        if self._door is not None:
            from std_msgs.msg import Float32
            self._door_pub = self._node.create_publisher(Float32, "/ground_truth/microwave_door", 10)
            if self._door_solid is not None:
                self._targets["microwave_door_pose"] = self._door_solid
                self._pubs["microwave_door_pose"] = self._node.create_publisher(
                    PoseStamped, "/ground_truth/microwave_door_pose", 10)
        self._log.info(f"ground_truth: publishing {sorted(self._targets)} "
                       f"at {rate:.0f} Hz")

    def step(self):
        self._count += 1
        if self._count % self._every:
            return
        t = self._robot.getTime()
        header = Header()
        header.frame_id = "map"
        header.stamp.sec = int(t)
        header.stamp.nanosec = int((t - int(t)) * 1e9)
        if self._door is not None:
            from std_msgs.msg import Float32
            self._door_pub.publish(Float32(data=float(self._door.getSFFloat())))
        for key, node in self._targets.items():
            p = node.getPosition()
            r = node.getOrientation()          # 3x3 row-major
            msg = PoseStamped(header=header)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
            q = _quat_from_matrix(r)
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = q
            self._pubs[key].publish(msg)
        rclpy.spin_once(self._node, timeout_sec=0)


    def _door_joint(self):
        mw = self._robot.getFromDef("MICROWAVE")
        if mw is None:
            return None
        try:
            children = mw.getBaseNodeField("children")     # the PROTO's internal Solid
            for i in range(children.getCount()):
                node = children.getMFNode(i)
                if node.getTypeName() == "HingeJoint":
                    # the door Solid itself as well: its pose on
                    # /ground_truth/microwave_door_pose says where the
                    # handle bar really is when the door is open
                    self._door_solid = node.getField("endPoint").getSFNode()
                    return node.getField("jointParameters").getSFNode().getField("position")
        except Exception as e:      # noqa: BLE001 -- best effort, diagnostics only
            self._log.warn(f"ground_truth: no microwave door joint: {e}")
        return None

    def _contacts(self, request, response):
        """Log the robot's contact points: world position, and the node's name
        when Webots will give it (nodes inside a PROTO often come back as
        None, so the position is the reliable part; z = 0.03 is a tyre)."""
        me = self._targets["robot"]
        points = me.getContactPoints(True)
        rows = []
        for cp in points:
            node = self._robot.getFromId(cp.node_id)
            name = "?"
            if node is not None:
                nm = node.getField("name")
                name = nm.getSFString() if nm is not None else node.getTypeName()
            rows.append(f"({cp.point[0]:.2f},{cp.point[1]:.2f},{cp.point[2]:.2f}) {name}")
        self._log.info(f"contacts: {len(points)} points: " + "; ".join(sorted(set(rows))))
        response.value = bool(points)
        return response

    def _snapshot(self, request, response):
        """Look at the robot from a bearing off its heading and export."""
        parts = request.value.split(",")
        path = parts[0]
        bearing = float(parts[1]) if len(parts) > 1 else 50.0
        dist = float(parts[2]) if len(parts) > 2 else 2.5
        # eye height, and how far up the robot to aim: the kitchen is small and
        # a fixed 1.6 m eye over a 0.8 m aim looks straight over the chassis
        # from anywhere there is room to stand
        eye = float(parts[3]) if len(parts) > 3 else 1.6
        aim = float(parts[4]) if len(parts) > 4 else 0.8
        # optionally a point to look at instead of the robot (x, y, z in the
        # world): the hand on a handle is 0.7 m from the robot's centre
        view = self._robot.getFromDef("VIEWPOINT")
        if view is None:
            response.success = False
            self._log.error("snapshot: no DEF VIEWPOINT in the world")
            return response
        x, y, z = self._targets["robot"].getPosition()
        yaw = math.atan2(self._targets["robot"].getOrientation()[3],
                         self._targets["robot"].getOrientation()[0])
        if len(parts) > 7:
            x, y, z = float(parts[5]), float(parts[6]), float(parts[7]) - aim
        a = yaw + math.radians(bearing)
        px, py, pz = x + dist * math.cos(a), y + dist * math.sin(a), eye
        # look from (px,py,pz) at (x, y, z + 0.8). A Webots Viewpoint, like
        # its cameras, looks along its +x with +y left and +z up (checked
        # against this world's own Viewpoint), so the frame is forward /
        # left / up and the rotation is its axis-angle.
        fx, fy, fz = x - px, y - py, (z + aim) - pz
        n = math.sqrt(fx * fx + fy * fy + fz * fz)
        fx, fy, fz = fx / n, fy / n, fz / n
        lx, ly, lz = -fy, fx, 0.0                     # left = up x forward
        n = math.hypot(lx, ly)
        lx, ly = lx / n, ly / n
        ux, uy, uz = (fy * lz - fz * ly, fz * lx - fx * lz, fx * ly - fy * lx)
        m = [[fx, lx, ux], [fy, ly, uy], [fz, lz, uz]]      # columns: x, y, z
        qx, qy, qz, qw = _quat_from_matrix([m[0][0], m[0][1], m[0][2],
                                            m[1][0], m[1][1], m[1][2],
                                            m[2][0], m[2][1], m[2][2]])
        angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
        s = math.sqrt(max(1e-12, 1 - qw * qw))
        view.getField("position").setSFVec3f([px, py, pz])
        view.getField("orientation").setSFRotation([qx / s, qy / s, qz / s, angle])
        self._robot.step(int(self._step_ms))
        self._robot.exportImage(path, 95)
        response.success = True
        self._log.info(f"snapshot: exported {path}")
        return response


def _quat_from_matrix(r):
    """(x, y, z, w) from a row-major 3x3 rotation matrix."""
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = r
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
