"""Bring up all of mrRobot: simulator, driver, state publisher, controllers,
localization, Nav2, MoveIt, perception, RViz -- and optionally the errand.

    ros2 launch mrrobot_bringup mrRobot.launch.py
    ros2 launch mrrobot_bringup mrRobot.launch.py mission:=true
    ros2 launch mrrobot_bringup mrRobot.launch.py localization:=slam nav:=false
    ros2 launch mrrobot_bringup mrRobot.launch.py headless:=true
    ros2 launch mrrobot_bringup mrRobot.launch.py gui:=false

RViz opens by default (rviz:=false for none) on mrrobot_description/rviz/
mrRobot.rviz: the map and costmaps, Nav2's plans, the AMCL cloud, the scan,
the robot, MoveIt's planning scene and every arm trajectory as it is planned,
the object memory's markers, the mission's caption and target, the
simulator's ground truth, and the four cameras.

Arguments:
    localization  amcl (default; needs mrrobot_navigation/maps/kitchen.yaml),
                  slam (map the room), or none (no map->odom at all)
    nav           the Nav2 servers                       (default true)
    moveit        move_group                             (default true)
    perception    colour detector + object memory        (default true)
    mission       run the errand from mission_file once the stack is up
                                                        (default false)
    mission_file  YAML the mission interprets (default kitchen_errand.yaml)
    gui           Webots' 3D window (default true; false keeps real time
                  and RViz, the cameras still render off-screen)
    headless      Webots with no rendering in fast mode, no RViz -- for CI

How the pieces fit together:

  WebotsLauncher starts Webots on the kitchen world. The mrRobot node in that
  world has controller "<extern>", so Webots waits for something to claim it;
  WebotsController is that something. It reads the <webots> block in the URDF to
  learn which devices to publish, and loads webots_ros2_control, which presents
  every motor as a ros2_control joint. From there the standard controllers --
  diff_drive_controller for the base, joint_trajectory_controller for the arms --
  work exactly as they would on real hardware.

  The controllers are chained off the driver's start rather than fired at
  once with it: a spawner that runs before controller_manager exists just
  times out, and the failure looks like a missing controller rather than a
  race. Nav2, MoveIt, perception and the mission wait for the controllers in
  turn, so the heavy servers configure on a quieter machine.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, Command, LaunchConfiguration,
                                  NotSubstitution, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher


def equals(config, value):
    """'true' when the launch configuration equals value."""
    return PythonExpression(["'", config, "' == '", value, "'"])


def webots_mode(headless):
    """Webots run mode: fast when headless, realtime otherwise."""
    return PythonExpression(["'fast' if '", headless, "' == 'true' else 'realtime'"])


def generate_launch_description():
    description_pkg = FindPackageShare("mrrobot_description")
    webots_pkg = FindPackageShare("mrrobot_webots")
    control_pkg = FindPackageShare("mrrobot_control")

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("rviz")
    run_mission = LaunchConfiguration("mission")
    mission_file = LaunchConfiguration("mission_file")
    localization = LaunchConfiguration("localization")
    use_nav = LaunchConfiguration("nav")
    use_moveit = LaunchConfiguration("moveit")
    use_perception = LaunchConfiguration("perception")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")

    args = [
        DeclareLaunchArgument(
            "world", default_value="mrRobot.wbt",
            description="World file inside mrrobot_webots/worlds."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with the mrRobot layout."),
        DeclareLaunchArgument(
            "mission", default_value="false",
            description="Run the errand in mission_file once everything is up."),
        DeclareLaunchArgument(
            "mission_file",
            default_value=PathJoinSubstitution(
                [control_pkg, "config", "missions", "kitchen_errand.yaml"]),
            description="Mission YAML for mrrobot_control's interpreter."),
        DeclareLaunchArgument(
            "localization", default_value="amcl",
            choices=["amcl", "slam", "none"],
            description="What publishes map->odom."),
        DeclareLaunchArgument(
            "nav", default_value="true", description="Start the Nav2 servers."),
        DeclareLaunchArgument(
            "moveit", default_value="true", description="Start move_group."),
        DeclareLaunchArgument(
            "perception", default_value="true",
            description="Start the colour detector and object memory."),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Webots' 3D window; false renders only the cameras."),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Webots without rendering, in fast mode, and no RViz."),
    ]

    # Two different forms of the same model, for two different consumers:
    #   the driver wants a PLAIN URDF on disk (it does not run xacro), which
    #   mrrobot_description expands at build time;
    #   robot_state_publisher wants the URDF text as a parameter.
    robot_description_xacro = PathJoinSubstitution(
        [description_pkg, "urdf", "mrRobot.urdf.xacro"])
    # The driver needs the <webots> and <ros2_control> blocks, so unlike
    # view_robot.launch.py this expansion keeps both switched on.
    #
    # ParameterValue(..., value_type=str) is not decoration: without it launch
    # tries to infer the parameter's type by parsing the expanded URDF as YAML,
    # which fails on the very first line and aborts the whole launch.
    robot_description = ParameterValue(
        Command(["xacro ", robot_description_xacro]), value_type=str)

    # ros2_supervisor=True appends a "Ros2Supervisor" robot to the world copy.
    # That robot is what publishes /clock -- the driver does not -- and without
    # /clock every node running on sim time sits at t=0, controller_manager
    # never runs an update, and each controller switch times out after 5 s.
    #
    # Its own launcher is NOT used, though: it hardcodes an ipc URL, and this
    # Webots offers no ipc endpoint (/tmp/webots stays empty). It is replaced
    # below by the same node with a tcp WEBOTS_CONTROLLER_URL.
    # headless: --no-rendering --minimize --batch --mode=fast. The sensors
    # still render (Webots does that off-screen), only the 3D view is gone.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([webots_pkg, "worlds", world]),
        ros2_supervisor=True,
        gui=AndSubstitution(gui, NotSubstitution(headless)),
        mode=webots_mode(headless),
    )

    supervisor = Node(
        package="webots_ros2_driver",
        executable="ros2_supervisor.py",
        namespace="Ros2Supervisor",
        remappings=[("/Ros2Supervisor/clock", "/clock")],
        output="screen",
        additional_env={
            "WEBOTS_CONTROLLER_URL": "tcp://127.0.0.1:1234/Ros2Supervisor",
            "WEBOTS_HOME": get_package_prefix("webots_ros2_driver"),
        },
        respawn=True,
    )

    # The driver is built by hand rather than with WebotsController, for two
    # reasons, both found by running it:
    #
    # 1. PROTOCOL. WebotsController picks ipc on plain Linux with no way to
    #    override it, but this Webots never creates an ipc endpoint -- /tmp/webots
    #    stays empty and the driver just retries for 50 s and gives up. Over tcp
    #    on the same port it connects immediately.
    #
    # 2. PARAMS FILE. WebotsController only forwards a parameters entry as
    #    --params-file when it is a plain `str`; a PathJoinSubstitution is
    #    silently dropped. Without the file the controller_manager comes up with
    #    no update_rate and aborts with "expected [integer] got [not set]".
    #    Hence get_package_share_directory here instead of a substitution.
    controllers_yaml = os.path.join(
        get_package_share_directory("mrrobot_control"), "config",
        "mrRobot_controllers.yaml")
    urdf_file = os.path.join(
        get_package_share_directory("mrrobot_description"), "urdf", "mrRobot.urdf")
    webots_controller_script = os.path.join(
        get_package_share_directory("webots_ros2_driver"), "scripts",
        "webots-controller")

    driver = ExecuteProcess(
        name="webots_controller_mrRobot",
        output="screen",
        cmd=[
            webots_controller_script,
            "--robot-name=mrRobot",
            "--protocol=tcp",
            "--ip-address=127.0.0.1",
            "--port=1234",
            "ros2", "--ros-args",
            "-p", ["robot_description:=", urdf_file],
            "-p", "use_sim_time:=true",
            "--params-file", controllers_yaml,
        ],
        # The controller library must come from webots_ros2_driver, not from the
        # Webots install; this is what WebotsController does too.
        additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver")},
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    # One process brings the controllers up in this order (see
    # mrrobot_control/spawn_controllers.py: controller_manager's own spawner
    # dies when its `load` reply is lost, which it is one start in ten).
    # joint_state_broadcaster first: without it nothing downstream sees a
    # joint_states topic, and RViz shows the robot collapsed at the origin.
    controllers = Node(
        package="mrrobot_control",
        executable="spawn_controllers",
        output="screen",
        arguments=["joint_state_broadcaster", "base_controller", "lift_controller",
                   "head_controller", "left_arm_controller", "right_arm_controller",
                   "left_gripper_controller", "right_gripper_controller"],
    )

    # The robot model as parameters too: MoveIt's displays (the planning
    # scene, the trajectory animation) load it from the node's own
    # robot_description, not from the topic.
    # The empty snap variables: a terminal inside a snap (VS Code's) exports
    # GTK_PATH, GIO_MODULE_DIR, LOCPATH and GSETTINGS_SCHEMA_DIR pointing
    # into the snap, and rviz2 started from it dies loading the snap's
    # libpthread (measured: "undefined symbol: __libc_pthread_init").
    # Nothing else here minds them.
    with open(os.path.join(get_package_share_directory("mrrobot_moveit_config"),
                           "config", "mrRobot.srdf")) as f:
        srdf = f.read()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(AndSubstitution(use_rviz, NotSubstitution(headless))),
        arguments=["-d", PathJoinSubstitution(
            [description_pkg, "rviz", "mrRobot.rviz"])],
        parameters=[{"use_sim_time": True,
                     "robot_description": robot_description,
                     "robot_description_semantic": srdf}],
        additional_env={"GTK_PATH": "", "GIO_MODULE_DIR": "", "LOCPATH": "",
                        "GSETTINGS_SCHEMA_DIR": ""},
    )

    def include(pkg, launch_file, condition, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare(pkg), "launch", launch_file])),
            condition=condition,
            launch_arguments=(arguments or {}).items(),
        )

    # Not optional: with enable_odom_tf off in the controller the EKF in here
    # is the only publisher of odom->base_link, and the scan filter is what
    # keeps the robot's own chassis out of every map and costmap.
    sensors = include("mrrobot_navigation", "sensors.launch.py", None)
    amcl = include("mrrobot_navigation", "localization.launch.py",
                   IfCondition(equals(localization, "amcl")))
    slam = include("mrrobot_navigation", "slam.launch.py",
                   IfCondition(equals(localization, "slam")))
    nav2 = include("mrrobot_navigation", "nav2.launch.py", IfCondition(use_nav))
    moveit = include("mrrobot_moveit_config", "move_group.launch.py",
                     IfCondition(use_moveit))
    perception = include("mrrobot_perception", "perception.launch.py",
                         IfCondition(use_perception))

    posture = Node(
        package="mrrobot_control",
        executable="posture",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    mission = Node(
        package="mrrobot_control",
        executable="mission",
        output="screen",
        condition=IfCondition(run_mission),
        parameters=[{"use_sim_time": True, "mission_file": mission_file}],
    )

    return LaunchDescription([
        *args,
        webots,
        supervisor,
        driver,
        state_publisher,
        rviz,
        # Wait for the driver: it is the process that creates controller_manager.
        RegisterEventHandler(OnProcessStart(
            target_action=driver,
            on_start=[controllers, sensors, amcl, slam],
        )),
        # ... and for the controllers, before the heavy servers: started
        # together with twenty other nodes, Nav2's controller_server took so
        # long to configure that its reply to the lifecycle manager timed
        # out and Nav2 never came up, one start in twelve (measured).
        RegisterEventHandler(OnProcessExit(
            target_action=controllers,
            on_exit=[nav2, moveit, perception, posture, mission],
        )),
    ])
