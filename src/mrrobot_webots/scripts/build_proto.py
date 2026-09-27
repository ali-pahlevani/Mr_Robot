#!/usr/bin/env python3
"""Generate MrRobot.proto from the expanded URDF.

    build_proto.py --urdf mrRobot.urdf --out <dir>

writes <dir>/MrRobot.proto and <dir>/meshes/... . Runs at build time from
mrrobot_webots/CMakeLists.txt, so the simulator model is always the URDF's:
every joint becomes a Webots motor of the same name with a "<joint>_sensor"
PositionSensor (urdf2webots' convention, and webots_ros2_control's), every
link's mass, inertia, visuals and collision meshes come across, and the
hand-written PROTO this replaced -- which had to be kept in step with the URDF
by hand -- is gone.

What this adds on top of urdf2webots:

  * Frame-only links (no mass, no collision: sensor frames, TCPs, the
    footprint) are pruned first, remembering their pose relative to the
    nearest solid ancestor. urdf2webots would otherwise drop them itself, and
    when such a link has several children it re-parents only one of them.
  * Sensors. urdf2webots knows nothing of Webots devices; each sensor frame
    in SENSORS gets a Pose at its URDF pose inside its parent's Solid, with
    the device nodes. Housings are the URDF's own visuals (the sensor links
    carry mass and collision, so they survive as Solids).
  * Collision meshes with a scale. Webots cannot scale a bounding Mesh, so
    OpenFleX's millimetre hand meshes and mirrored left-arm meshes are baked
    into new STL files at unit scale.
  * Visual STL meshes over VISUAL_TRI_LIMIT triangles are decimated by
    vertex clustering (snapping to a VISUAL_GRID grid) into a copy the
    simulator renders instead: Webots renders the scene once per camera per
    step, and the lift column's 212k triangles were a third of the frame
    time. RViz still gets the originals through the URDF.
  * Collision meshes over COLLISION_TRI_LIMIT triangles become their
    bounding box. OpenFleX collides the lift column with its full 212k
    triangle visual mesh; as an ODE trimesh that alone pulled the simulation
    to 0.6x real time. The column is a box anyway.
  * The finger motors' force limits per hand (see FINGER_FORCE).
  * Hard stops on every joint at its URDF limits. urdf2webots only sets the
    motor's minPosition/maxPosition, which clamp COMMANDS; a contact can
    still push the joint past them, and once it has, the motor cannot bring
    it back (measured: a wrist at -2.36 rad in a +-0.75 joint after the
    hand met the worktop, and every plan refused from then on).
  * Contact materials on the wheels and fingers (the world defines them),
    castShadows FALSE on meshes over Webots' shadow limit, and the fingers'
    83k-triangle DAE swapped for the 9k STL OpenFleX also ships.
  * Mesh files copied next to the PROTO and referenced relatively, so the
    installed PROTO is self-contained.
  * The Husky's own colours. Clearpath's DAEs carry no usable material and
    come out black as CadShapes; they are painted the way the hand-written
    PROTO painted them (bumpers and frame in the accent, shell in white,
    rails and plate in the trim).
"""

import argparse
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

# --------------------------------------------------------------- sensors
# Webots device nodes per URDF sensor frame. Frame conventions match: a Webots
# Camera/RangeFinder/Lidar looks along its +x, the URDF camera frames are
# x-forward.


def camera(name, w, h, fov, near):
    return (f'Camera {{ name "{name}" width {w} height {h} fieldOfView {fov} '
            f'near {near} }}')


def rangefinder(name, w, h, fov, lo, hi):
    return (f'RangeFinder {{ name "{name}" width {w} height {h} fieldOfView {fov} '
            f'minRange {lo} maxRange {hi} }}')


SENSORS = {
    "lidar_link": [
        'Lidar { name "lidar" horizontalResolution 270 fieldOfView 3.3161256 '
        'numberOfLayers 1 minRange 0.12 maxRange 25 }',
    ],
    "imu_link": [
        'InertialUnit { name "imu" }',
        'Gyro { name "gyro" }',
        'Accelerometer { name "accelerometer" }',
    ],
    "base_link": [
        'GPS { name "gps" }',
    ],
    # The base's D455, the same unit as the head's and now the same picture:
    # it was 320x320 at 1.57, a square frame of a 90 degree cone, which is
    # neither the device's field nor a 4:3 image, so anything measured off it
    # was measured off a lens the robot does not have.
    "front_camera_link": [
        camera("front_camera", 640, 480, 1.518, 0.05),
        rangefinder("front_depth", 640, 480, 1.518, 0.05, 8.0),
    ],
    # The head's D455: 87 deg (its depth field), 640x480 as it streams.
    "head_camera_link": [
        camera("head_camera", 640, 480, 1.518, 0.05),
        rangefinder("head_depth", 640, 480, 1.518, 0.1, 6.0),
    ],
    "left_wrist_camera_link": [
        camera("left_wrist_camera", 640, 480, 1.047, 0.05),
        rangefinder("left_wrist_depth", 640, 480, 1.047, 0.05, 5.0),
    ],
    "right_wrist_camera_link": [
        camera("right_wrist_camera", 640, 480, 1.047, 0.05),
        rangefinder("right_wrist_depth", 640, 480, 1.047, 0.05, 5.0),
    ],
}

# Clearpath's a200 DAEs, painted: mesh file -> baseColor. The shell's green is
# the arms' own (openarmx_description/meshes/arm/v10/visual/link2.dae): the base
# and the body are one machine, not two. The DAEs carry the same colours in
# their own materials so that RViz, which ignores the URDF's <material> when a
# mesh brings one, shows the same robot as Webots.
PAINT = {
    "base_link.dae": "0.15 0.65 0.70",
    "top_chassis.dae": "0.93 0.93 0.95",
    "user_rail.dae": "0.13 0.13 0.15",
    "top_plate.dae": "0.93 0.93 0.95",
    "wheel.dae": "0.13 0.13 0.15",
}

CONTACT_MATERIALS = {
    r"_wheel_link$": "mrRobot wheel",
    r"_finger$": "mrRobot finger",
}

# Visuals too heavy for Webots, and what to use instead.
SHADOW_LIMIT = 21845
COLLISION_TRI_LIMIT = 15000
VISUAL_TRI_LIMIT = 40000
VISUAL_GRID = 0.004          # metres, in the mesh's own units after scale

# --------------------------------------------------------------- maths


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(m):
    angle = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2)))
    if angle < 1e-9:
        return "0 0 1 0"
    if abs(angle - math.pi) < 1e-6:
        # axis from the largest diagonal term
        x = math.sqrt(max(0.0, (m[0][0] + 1) / 2))
        y = math.sqrt(max(0.0, (m[1][1] + 1) / 2))
        z = math.sqrt(max(0.0, (m[2][2] + 1) / 2))
        # fix signs
        if m[0][1] < 0:
            y = -y
        if m[0][2] < 0:
            z = -z
        return f"{x:.6f} {y:.6f} {z:.6f} 3.141593"
    s = 2 * math.sin(angle)
    ax = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    return f"{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f} {angle:.6f}"


def origin_of(joint):
    o = joint.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy_matrix(*rpy)


# --------------------------------------------------------------- URDF prep


def is_solid(link):
    return link.find("inertial") is not None or link.find("collision") is not None


def prune_frames(robot):
    """Remove links with neither mass nor collision, returning for each the
    (solid ancestor, translation, rotation matrix) of its frame."""
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = list(robot.findall("joint"))
    parent_joint = {j.find("child").get("link"): j for j in joints}
    frames = {}

    def pose_in_solid_ancestor(name):
        t, R = [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        while name in parent_joint:
            j = parent_joint[name]
            xyz, Rj = origin_of(j)
            t = [xyz[i] + mat_vec(Rj, t)[i] for i in range(3)]
            R = mat_mul(Rj, R)
            name = j.find("parent").get("link")
            if is_solid(links[name]):
                return name, t, R
        return name, t, R      # reached the root (itself a frame)

    for name, link in list(links.items()):
        if is_solid(link):
            continue
        if name in parent_joint:
            frames[name] = pose_in_solid_ancestor(name)
            robot.remove(parent_joint[name])
        robot.remove(link)
    # the root frame (base_footprint) had children: their joints now dangle
    for j in list(robot.findall("joint")):
        if j.find("parent").get("link") not in links or \
                not is_solid(links[j.find("parent").get("link")]):
            robot.remove(j)
    return frames


def bake_stl(src, dst, scale):
    """Write an STL with the scale (possibly negative = mirrored) applied."""
    data = open(src, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        nums = re.findall(rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data)
        tris = [tuple(float(x) for x in v) for v in nums]
        tris = [tris[i:i + 3] for i in range(0, len(tris), 3)]
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tris = []
        for i in range(n):
            rec = struct.unpack("<12fH", data[84 + i * 50:84 + (i + 1) * 50])
            tris.append([rec[3:6], rec[6:9], rec[9:12]])
    flip = scale[0] * scale[1] * scale[2] < 0
    out = []
    for tri in tris:
        v = [tuple(c * s for c, s in zip(p, scale)) for p in tri]
        if flip:
            v = [v[0], v[2], v[1]]
        out.append(v)
    write_stl(dst, out)


def stl_bounds(path, scale=(1, 1, 1)):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        pts = [tuple(float(x) for x in v) for v in
               re.findall(rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data)]
    else:
        n = struct.unpack("<I", data[80:84])[0]
        pts = []
        for i in range(n):
            rec = struct.unpack("<12f", data[84 + i * 50:84 + i * 50 + 48])
            pts += [rec[3:6], rec[6:9], rec[9:12]]
    lo = [min(p[i] * scale[i] for p in pts) for i in range(3)]
    hi = [max(p[i] * scale[i] for p in pts) for i in range(3)]
    lo, hi = [min(a, b) for a, b in zip(lo, hi)], [max(a, b) for a, b in zip(lo, hi)]
    return lo, hi


def read_stl(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        nums = re.findall(rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data)
        pts = [tuple(float(x) for x in v) for v in nums]
        return [pts[i:i + 3] for i in range(0, len(pts), 3)]
    n = struct.unpack("<I", data[80:84])[0]
    return [[struct.unpack("<3f", data[84 + i * 50 + 12 + k * 12:84 + i * 50 + 24 + k * 12])
             for k in range(3)] for i in range(n)]


def write_stl(path, tris):
    out = bytearray(b"decimated by build_proto.py".ljust(80, b"\0"))
    out += struct.pack("<I", len(tris))
    for tri in tris:
        a, b, c = tri
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        nrm = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(x * x for x in nrm)) or 1.0
        out += struct.pack("<3f", *(x / length for x in nrm))
        for p in tri:
            out += struct.pack("<3f", *p)
        out += struct.pack("<H", 0)
    open(path, "wb").write(bytes(out))


def decimate_stl(src, dst, grid):
    """Vertex clustering: snap every vertex to a grid, drop collapsed triangles."""
    tris = read_stl(src)
    out = []
    for tri in tris:
        q = [tuple(round(c / grid) * grid for c in p) for p in tri]
        if q[0] != q[1] and q[1] != q[2] and q[0] != q[2]:
            out.append(q)
    write_stl(dst, out)
    return len(tris), len(out)


def tri_count(path):
    data = open(path, "rb").read()
    if path.lower().endswith(".stl"):
        if data[:5] == b"solid" and b"facet" in data[:400]:
            return data.count(b"facet normal")
        return struct.unpack("<I", data[80:84])[0]
    text = data.decode(errors="ignore")
    return sum(int(x) for x in re.findall(r'<(?:triangles|polylist)[^>]*count="(\d+)"', text))


def resolve(package_url):
    from ament_index_python.packages import get_package_share_directory
    rest = package_url[len("package://"):]
    pkg, path = rest.split("/", 1)
    return os.path.join(get_package_share_directory(pkg), path)


def prepare_urdf(robot, out_dir):
    """Edit the URDF copy fed to urdf2webots. Returns the pruned frames."""
    for tag in ("ros2_control", "webots"):
        for el in robot.findall(tag):
            robot.remove(el)
    frames = prune_frames(robot)
    baked = os.path.join(out_dir, "meshes", "baked")
    os.makedirs(baked, exist_ok=True)
    for link in robot.findall("link"):
        for coll in link.findall("collision"):
            mesh = coll.find("geometry/mesh")
            if mesh is None:
                continue
            scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
            src = resolve(mesh.get("filename"))
            if src.lower().endswith(".stl") and tri_count(src) > COLLISION_TRI_LIMIT:
                lo, hi = stl_bounds(src, scale)
                geom = coll.find("geometry")
                geom.remove(mesh)
                box = ET.SubElement(geom, "box")
                box.set("size", " ".join(f"{hi[i] - lo[i]:.4f}" for i in range(3)))
                origin = coll.find("origin")
                if origin is None:
                    origin = ET.SubElement(coll, "origin")
                    origin.set("rpy", "0 0 0")
                oxyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                origin.set("xyz", " ".join(f"{oxyz[i] + (lo[i] + hi[i]) / 2:.4f}" for i in range(3)))
                continue
            if not mesh.get("scale"):
                continue
            if scale == [1.0, 1.0, 1.0]:
                del mesh.attrib["scale"]
                continue
            tag = "_".join(f"{s:g}" for s in scale).replace("-", "m").replace(".", "p")
            dst = os.path.join(baked, os.path.splitext(os.path.basename(src))[0] + f"_{tag}.stl")
            if not os.path.exists(dst):
                bake_stl(src, dst, scale)
            mesh.set("filename", dst)
            del mesh.attrib["scale"]
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            fn = mesh.get("filename")
            if fn.lower().endswith(".stl"):
                src = resolve(fn)
                if tri_count(src) > VISUAL_TRI_LIMIT:
                    scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
                    grid = VISUAL_GRID / abs(scale[0])
                    dst = os.path.join(baked, os.path.splitext(os.path.basename(src))[0] + "_lod.stl")
                    if not os.path.exists(dst):
                        before, after = decimate_stl(src, dst, grid)
                        print(f"build_proto: decimated {os.path.basename(src)} {before} -> {after} triangles")
                    mesh.set("filename", dst)
            if fn.endswith("openarmx_hand/visual/finger.dae"):
                mesh.set("filename", fn[:-4] + ".stl")
    return frames


# --------------------------------------------------------------- PROTO edits


def find_solid(text, link):
    m = re.search(rf"^(\s*)(?:endPoint )?DEF {re.escape(link)} Solid \{{\n", text, re.M)
    if m is None:
        # the root link is the Robot node itself
        m = re.search(r"^(\s*)Robot \{\n", text, re.M)
    if m is None:
        raise SystemExit(f"build_proto: no Solid for link {link} in the PROTO")
    return m


def inject_children(text, link, nodes, translation="0 0 0", rotation="0 0 1 0"):
    m = find_solid(text, link)
    indent = m.group(1) + "  "
    body = "\n".join(indent + "    " + n for n in nodes)
    pose = (f"{indent}  Pose {{\n{indent}    translation {translation}\n"
            f"{indent}    rotation {rotation}\n{indent}    children [\n{body}\n"
            f"{indent}    ]\n{indent}  }}\n")
    cm = re.compile(r"^" + re.escape(indent) + r"children \[\n", re.M)
    c = cm.search(text, m.end())
    if c is None:
        # a Solid without children: add the field
        return text[:m.end()] + f"{indent}children [\n{pose}{indent}]\n" + text[m.end():]
    return text[:c.end()] + pose + text[c.end():]


def add_field(text, link, field_line):
    m = find_solid(text, link)
    return text[:m.end()] + m.group(1) + "  " + field_line + "\n" + text[m.end():]


def relocate_meshes(text, out_dir):
    """Copy every mesh the PROTO references next to it; rewrite the URLs."""
    def repl(m):
        src = m.group(1)
        if not os.path.isabs(src):
            return m.group(0)
        share = re.search(r"/share/([^/]+)/(.*)$", src)
        rel = os.path.join(share.group(1), share.group(2)) if share else \
            os.path.join("baked", os.path.basename(src))
        dst = os.path.join(out_dir, "meshes", rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.realpath(src) != os.path.realpath(dst):
            shutil.copyfile(src, dst)
        return f'url "meshes/{rel}"'
    return re.sub(r'url "([^"]+)"', repl, text)


# The finger motors' force limits, per hand, over the URDF's 60 N. The
# left hand carries the jars: its fingers are effort-controlled springs
# (mrRobot_controllers.yaml), and a spring only centres a jar while it is
# UNDER its motor's cap; 120 N leaves room for a 100 N pinch, which a
# jolt through the robot (the other hand pulling the microwave door) no
# longer pops a jar out of (measured at 49 N: it did, twice). The right
# hand works the door: its stiff fingers on the handle bar at 60 N were
# that jolt, a bar pinched by its corner; 20 N is plenty for the pull
# (the bar is dragged by friction, 2 x 1.6 x 20 N, against a door that
# needs 5) and gentle on the door.
FINGER_FORCE = {"left": 120.0, "right": 20.0}


def set_finger_forces(text):
    for side, force in FINGER_FORCE.items():
        text = re.sub(rf'(name "openarmx_{side}_finger_joint[12]"\n\s*maxVelocity [\d.]+\n'
                      rf'(?:\s*maxPosition [\d.]+\n)?\s*)maxForce [\d.]+',
                      rf"\g<1>maxForce {force}", text)
    return text


def add_joint_stops(text):
    """minStop/maxStop on each HingeJoint/SliderJoint from its motor's range."""
    lines = text.split("\n")
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r"(\s*)jointParameters (HingeJointParameters|JointParameters) \{\s*$", line)
        if m:
            indent = m.group(1)
            # find the closing brace of this parameters block and the motor after it
            j = i + 1
            while not re.match(r"^" + re.escape(indent) + r"\}\s*$", lines[j]):
                j += 1
            block = "\n".join(lines[i:j + 1])
            k = j + 1
            lo = hi = None
            while k < len(lines) and k < j + 40:
                mm = re.search(r"minPosition ([-\d.e+]+)", lines[k])
                if mm:
                    lo = float(mm.group(1))
                mm = re.search(r"maxPosition ([-\d.e+]+)", lines[k])
                if mm:
                    hi = float(mm.group(1))
                if re.search(r"endPoint ", lines[k]):
                    break
                k += 1
            if hi is not None and lo is None:
                lo = 0.0                     # urdf2webots omits a zero minPosition
            if lo is not None and hi is not None and hi > lo and "minStop" not in block:
                margin = 0.02
                out.extend(lines[i:j])
                out.append(f"{indent}  minStop {lo - margin:.4f}")
                out.append(f"{indent}  maxStop {hi + margin:.4f}")
                out.append(lines[j])
                i = j + 1
                continue
        out.append(line)
        i += 1
    return "\n".join(out)


def paint_husky(text):
    """CadShape { url "...a200/x.dae" } -> a Shape with a PBRAppearance."""
    def repl(m):
        indent, defn, url = m.group(1), m.group(2) or "", m.group(3)
        color = PAINT[os.path.basename(url)]
        return (f"{indent}{defn}Shape {{\n{indent}  appearance PBRAppearance {{ baseColor {color} "
                f"roughness 0.4 metalness 0.1 }}\n{indent}  geometry Mesh {{ url \"{url}\" }}\n{indent}}}")
    pattern = r'^(\s*)(DEF \S+ )?CadShape \{\n\s*url "([^"]*a200/[^"]+\.dae)"\n\s*\}'
    return re.sub(pattern, repl, text, flags=re.M)


def shadows_off_for_heavy_meshes(text, out_dir):
    """castShadows FALSE on every Shape whose mesh is over Webots' limit."""
    lines = text.split("\n")
    for i, line in enumerate(lines):
        m = re.match(r'(\s*)url "(meshes/[^"]+)"', line)
        if not m:
            continue
        path = os.path.join(out_dir, m.group(2))
        if not os.path.exists(path) or tri_count(path) <= SHADOW_LIMIT:
            continue
        # walk back to the enclosing Shape
        for j in range(i, -1, -1):
            sm = re.match(r"(\s*)(?:DEF \S+ )?Shape \{\s*$", lines[j])
            if sm:
                lines[j] += "\n" + sm.group(1) + "  castShadows FALSE"
                break
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="MrRobot")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    text = open(args.urdf).read()
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)     # the importer reads comments
    robot = ET.fromstring(text)
    frames = prepare_urdf(robot, args.out)

    with tempfile.TemporaryDirectory() as tmp:
        urdf_copy = os.path.join(tmp, "mrRobot.urdf")
        ET.ElementTree(robot).write(urdf_copy)
        proto_tmp = os.path.join(tmp, args.name + ".proto")
        sys.path.append(os.path.join(os.path.dirname(subprocess.check_output(
            ["python3", "-c", "import webots_ros2_importer.urdf2proto as m; print(m.__file__)"]
        ).decode().strip()), "urdf2webots"))
        from urdf2webots.importer import convertUrdfFile
        # robotName= would switch the importer to "Robot node string" mode;
        # the PROTO's name comes from the output file name.
        convertUrdfFile(input=urdf_copy, output=proto_tmp,
                        linkToDef=True, targetVersion="R2025a")
        proto = open(proto_tmp).read()

    for frame, nodes in SENSORS.items():
        if frame in frames:
            parent, t, R = frames[frame]
            proto = inject_children(proto, parent, nodes,
                                    f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}", axis_angle(R))
        else:                       # a solid link: the device sits at its origin
            proto = inject_children(proto, frame, nodes)

    for pattern, material in CONTACT_MATERIALS.items():
        for link in re.findall(r"DEF (\S+) Solid \{", proto):
            if re.search(pattern, link):
                proto = add_field(proto, link, f'contactMaterial "{material}"')

    proto = relocate_meshes(proto, args.out)
    proto = add_joint_stops(proto)
    proto = set_finger_forces(proto)
    proto = paint_husky(proto)
    proto = shadows_off_for_heavy_meshes(proto, args.out)
    header = ("# GENERATED by mrrobot_webots/scripts/build_proto.py from the URDF in\n"
              "# mrrobot_description. Do not edit; change the URDF.\n")
    proto = proto.replace("PROTO " + args.name, header + "PROTO " + args.name, 1)
    open(os.path.join(args.out, args.name + ".proto"), "w").write(proto)
    print(f"build_proto: wrote {args.out}/{args.name}.proto "
          f"({len(frames)} frames pruned, {len(SENSORS)} sensor frames)")


if __name__ == "__main__":
    main()
