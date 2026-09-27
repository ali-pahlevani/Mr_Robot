# The kitchen map

`kitchen.{pgm,yaml}` is what the costmaps (and RViz) use. `kitchen_raw.*` is
the untouched slam_toolbox save it was made from, and `kitchen_surface.*` is
`kitchen` thinned to the cells the lidar can see -- the map AMCL localizes on
(`scripts/surface_map.py`, see "Why AMCL gets its own map" below).
`kitchen_reloc.*` is the scan relocalizer's (see "Why the scan matcher gets its
own map").

## How it was made

```bash
ros2 launch mrrobot_bringup mrRobot.launch.py localization:=slam nav:=false \
    moveit:=false perception:=false
ros2 run mrrobot_navigation map_kitchen.py            # drives the tour
ros2 run nav2_map_server map_saver_cli -f /tmp/raw --ros-args -p use_sim_time:=true
ros2 run mrrobot_navigation finish_map.py /tmp/raw.yaml src/mrrobot_navigation/maps/kitchen
```

## The map frame IS the Webots world frame

slam_toolbox starts its map frame where the robot starts, and the EKF takes
its heading from the IMU, so the SLAM frame is the world frame translated by
the spawn point (0.15, 0.20). `finish_map.py` adds that offset to the yaml
origin. Checked against the world file after the shift, using the occupied
cells in the saved map (a half-cell bias, 2.5 cm, is inherent):

| surface | map | world | tilt |
|---|---|---|---|
| east counter front | 1.445 | 1.45 | 0.0 deg |
| north counter front | 0.995 | 0.98 | 0.0 deg |
| south wall face | -3.74 | -3.82 | 0.2 deg |

(The south wall was only ever scanned from 2.5 m away; the mission never goes
there.)

So every world coordinate in the mission (jars, microwave, table, park lines)
is a map coordinate, and AMCL's initial pose is simply the spawn.

**The origin then moved by (-0.004, -0.020).** The scan matcher fits the
lidar's points to the centres of the occupied cells, and where a counter's
face falls inside its cells is down to where the grid happened to land: the
north counter's cells are centred 1.5 cm north of its face. Every accepted
match came out biased by the same amount wherever the robot stood -- over 72
matches in nine errands, (+0.40, +1.96) cm +- (0.24, 0.52) against the
simulator's ground truth -- and every fixture (the microwave's cavity, the
table's spot) inherited it: the honey went in 2.6 cm right of the cavity's
centre, run after run. With all three maps' origins moved by that bias, the
same scans replayed through the matcher come out at (-0.1, -0.1) cm. (A map
made again from a new save will sit differently on its grid: measure the
matcher against the truth before trusting it to a centimetre.)

**Two far walls were redrawn** in `kitchen_reloc` and `kitchen_surface`, at the
world file's faces: the south wall (y -3.82; the saved map had it ragged, 5-10
cm north, from being scanned at a grazing angle from 3 m away) and the wall
south of the east counter (x 1.15; saved 4 cm east). Facing south, a match
against them came out 9 cm wrong in y; with them redrawn it is right in y but
still 4-5 cm out in x (the dining set's scanned legs), so the errand does not
match facing south (`leg_match: false` on the jam station).

## What was painted in

The lidar plane is 0.26 m up and sees *legs*. Both tables' tops and the six
chairs' seats are drawn in as occupied: the planner must never send the
0.57 m base -- which fits between a table's legs -- and the 1.40 m torso on
top of it under a table, and 6 cm chair legs are hit by a 1 degree beam too
rarely for the costmaps to hold them. (Painted geometry is exactly the world
file's; `finish_map.py` has the list.)

## Why AMCL gets its own map

AMCL's likelihood field scores a beam by its distance to the nearest
occupied cell, so with walls 2-3 cells thick a beam ending anywhere inside
a wall scores as well as one ending on its face, and the estimate can slide
towards a wall by its thickness for free. Measured at the counter stations:
8-14 cm along the lane after every hop, which the scan matcher
(`scan_relocalize.py`) corrected each time at the cost of a re-approach.
`kitchen_surface` keeps only the occupied cells that border the free region
the robot drives in; the rest of every wall and the inside of every painted
piece are unknown, which the likelihood field treats as empty.

```bash
ros2 run mrrobot_navigation surface_map.py src/mrrobot_navigation/maps/kitchen.yaml \
    src/mrrobot_navigation/maps/kitchen_surface
```

## Why it is an odometry map

`config/slam_toolbox.yaml` runs with scan matching and loop closure off. With
them on, the first map had the east counter tilted 4.4 degrees, the west wall
11.7 and the south wall 3 -- and AMCL on it carried a 4-5 degree yaw bias and
10-25 cm of position error. The EKF's odometry (IMU heading, wheel distance)
brings the tour back to its start within 6 cm, which the 1 degree, 8.6 Hz
lidar cannot improve on in a room this size, so the map is built on it.

## Why two things had to be fixed before SLAM worked at all

Documented in `scripts/scan_fixer.py`: the Webots lidar scans clockwise
(negative `angle_increment`), which mirrored every scan for Karto and produced
rotated copies of the room stacked on each other; and its rear half sees the
robot's own chassis.

And in the base's centre of mass (`base_link`'s inertial in
`mrRobot.urdf.xacro`): with Clearpath's centre of mass under a front-heavy
upper body, a turn on the spot carried the robot 0.4 m sideways, invisible to
wheel odometry. The base's centre of mass is now balanced (measured against
supervisor ground truth) and a 90 degree spin drifts 4-10 cm.

## Why the scan matcher gets its own map

`kitchen` paints the dining table and its chairs as one solid block, because
the costmaps must not let a plan run under the table top the lidar cannot see.
But the lidar sees the table's legs and the chairs', inside that block, and
against it every scan match near the table came out 5-9 cm and was refused.
`kitchen_reloc` is `kitchen` with that rectangle (x -1.45..0.25, y
-2.70..-0.15) taken back from `kitchen_raw` -- the legs, as scanned -- and the
relocalizer matches against it (`/map_reloc`, `localization.launch.py`). It is
the same grid as `kitchen`, so the splice is cell for cell:

```python
out = kitchen.copy(); out[41:93, 28:63] = kitchen_raw[41:93, 28:63]
```

(A map made from `kitchen_raw` everywhere was tried for both AMCL and the
relocalizer: at the honey station it confirmed an 11 cm error rather than
correcting it. Only the dining set is taken from it.)

