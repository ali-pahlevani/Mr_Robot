"""Where the microwave's door face really is, read off the wrist camera.

The grip on the handle bar has a two-millimetre budget and the bar is only ten
millimetres thick (Oven.proto, with the microwave's [0.5, 1.2, 0.4] scale: the
bar's box spans door-local x 0.005..0.015 and the panel's 0.025..0, so the bar
stands 5 mm clear of a face the fingertips must not touch). The fixture in the
mission YAML is exact, but it is in the MAP, and the hand gets there through
AMCL: one to three centimetres. Aimed by the fixture alone the fingertips land
4.6 mm off the panel, so half a centimetre of localization error puts them
inside it, the fingers -- which slide, they do not rotate -- are stopped by the
panel before they have closed a centimetre, and the grip is logged as jammed.

The right wrist's D405 settles that. Its optical axis IS the hand's approach
axis and sits at the TCP's own height (mrRobot.urdf.xacro mounts it at hand
(0.0495, 0, 0.02) with rpy (0, -pi/2, 0), and the fingers close along hand y),
so the depth it reports along that axis is exactly the quantity the grip needs,
with no lever arm to get wrong. At the pre-grasp pose the panel is 0.1975 m
away, where one pixel is 0.36 mm.

The measurement is deliberately taken about the PRINCIPAL POINT rather than at
the projected handle: the point of it is to leave the fixture's error out.
"""

import math

import numpy as np

# The panel is a large flat plane; the bar is a 10 mm slab standing 5 mm in
# front of it, and past the door's top edge is the cavity, a quarter of a metre
# further back. The mode of a 2 mm histogram is the panel by a wide margin.
BIN = 0.002
PANEL_BAND = 0.006         # samples this close to the mode are the panel
CAVITY_BEYOND = 0.05       # anything further than the mode + this is not the door
BAR_NEAR = 0.019           # the bar's face sits mode - 0.015; +/- 4 mm of band
BAR_FAR = 0.011
MIN_PANEL = 300            # samples; the window holds ~19000
MIN_BAR = 40
SANE = 0.06                # the fixture may be this wrong before we disbelieve it


class PanelFit:
    """What one wrist frame says about the door in front of the hand."""

    def __init__(self, panel, n_panel, bar, n_bar, v_bar, bar_px=None):
        self.panel = panel          # metres along the optical axis
        self.n_panel = n_panel
        self.bar = bar              # None when the bar was not found
        self.n_bar = n_bar
        self.v_bar = v_bar          # the bar's centroid row, for the log
        # (u, v, depth) of every bar pixel, full-image coordinates: enough to
        # put the bar in 3-D without caring which way up the camera is
        self.bar_px = bar_px

    def __repr__(self):
        bar = "none" if self.bar is None else f"{self.bar:.4f} ({self.n_bar} px, row {self.v_bar})"
        return f"PanelFit(panel={self.panel:.4f} m, {self.n_panel} px, bar={bar})"


def panel_depth_from_patch(depth, cx, cy, half_u=80, half_v=60, expect=None):
    """Fit the door's face in a window about the principal point.

    `depth` is a (h, w) array of metres, non-finite where nothing was seen.
    Returns a PanelFit, or None when the window does not hold a door.
    """
    h, w = depth.shape
    u0, u1 = max(0, int(cx) - half_u), min(w, int(cx) + half_u + 1)
    v0, v1 = max(0, int(cy) - half_v), min(h, int(cy) + half_v + 1)
    patch = depth[v0:v1, u0:u1]
    good = np.isfinite(patch) & (patch > 0.05)
    if good.sum() < MIN_PANEL:
        return None
    z = patch[good]

    # the mode of the near half: the cavity is the only thing that could be
    # more numerous, and it is always further away, so cut it first
    near = z[z < z.min() + CAVITY_BEYOND * 4]
    counts, edges = np.histogram(near, bins=max(1, int((near.ptp() + BIN) / BIN) + 1))
    mode = edges[int(np.argmax(counts))] + BIN / 2

    on_door = z[z < mode + CAVITY_BEYOND]
    panel_hits = on_door[np.abs(on_door - mode) < PANEL_BAND]
    if panel_hits.size < MIN_PANEL:
        return None
    panel = float(np.median(panel_hits))
    if expect is not None and abs(panel - expect) > SANE:
        return None

    # the bar, as a mask over the window so its row can be reported too
    bar_mask = good & (patch > panel - BAR_NEAR) & (patch < panel - BAR_FAR)
    n_bar = int(bar_mask.sum())
    if n_bar >= MIN_BAR:
        rows, cols = np.nonzero(bar_mask)
        bar = float(np.median(patch[bar_mask]))
        v_bar = float(rows.mean()) + v0
        bar_px = (cols + u0, rows + v0, patch[bar_mask])
    else:
        bar, v_bar, bar_px = None, None, None
    return PanelFit(panel, int(panel_hits.size), bar, n_bar, v_bar, bar_px)


def to_camera(u, v, d, fx, fy, cx, cy):
    """Pixels and planar depth to points in the camera LINK's frame, which is
    Webots' convention (x along the view, y left, z up; image u right, v
    down), as color_detector.py deprojects."""
    return np.stack([d, -(u - cx) * d / fx, -(v - cy) * d / fy])


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class WristDepth:
    """The latest depth frame from one wrist, and the camera that took it.

    A frame is only handed out when it was taken AFTER a given time: the arm
    has just driven 12 cm and a frame from before that move would measure the
    place the hand came from. At 5 Hz that means waiting about 0.4 s.
    """

    def __init__(self, node, side):
        from sensor_msgs.msg import CameraInfo, Image
        from rclpy.qos import qos_profile_sensor_data
        self._node = node
        self._image = None
        self._info = None
        node.create_subscription(Image, f"/{side}_wrist_depth/image",
                                 self._on_image, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f"/{side}_wrist_camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)

    def _on_image(self, msg):
        self._image = msg

    def _on_info(self, msg):
        self._info = msg

    def latest(self, after=None):
        """(depth in metres, (fx, fy, cx, cy)), or None."""
        msg, info = self._image, self._info
        if msg is None or info is None or msg.encoding != "32FC1":
            return None
        if after is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= after:
                return None
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return depth, (info.k[0], info.k[4], info.k[2], info.k[5])


def quat_rotate(q, v):
    """Rotate v by the quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def bar_in_base(fit, intrinsics, cam_t, cam_q):
    """The bar's face in base_link, as the median of its back-projected
    pixels, or None. The camera's pose comes from TF, so this is right
    whichever way up the wrist is and whatever tilt the planner's tolerance
    left the hand with: the bar is where the pixels say, not where the
    optical axis happens to point."""
    if fit.bar_px is None:
        return None
    fx, fy, cx, cy = intrinsics
    u, v, d = fit.bar_px
    pts = quat_matrix(cam_q) @ to_camera(u.astype(float), v.astype(float), d, fx, fy, cx, cy)
    pts += np.asarray(cam_t, dtype=float).reshape(3, 1)
    return tuple(float(np.median(c)) for c in pts)
