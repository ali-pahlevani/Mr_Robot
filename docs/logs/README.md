# Run logs

| File | Contents |
|---|---|
| `errand_done.log` | The mission node's log of one complete run of the kitchen errand: every step, docking decision, scan match and arm move, ending with the per-step summary (all 16 steps `ok` on their first attempt). |

To produce one, launch the errand and keep the mission node's lines:

```bash
ros2 launch mrrobot_bringup mrRobot.launch.py mission:=true 2>&1 | tee run.log
grep "mrRobot_mission\]" run.log
```
