"""One launch for every domain.

  ros2 launch mobility_bringup mobility.launch.py domain:=mock|gazebo|unity|real \\
       input:=keyboard|joy|none  [include_sim:=true] [demo:=true]

Identical core nodes in every domain (swerve_kinematics, wheel_controller,
wheel_odometry, teleop); only the adapter and config/<domain>.yaml change.
Params load in order: geometry.yaml -> common.yaml -> <domain>.yaml -> extra_params.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ADAPTERS = {"mock": "mock_adapter", "gazebo": "gazebo_adapter",
            "unity": "unity_adapter", "real": "can_adapter"}
SIM_TIME = {"mock": False, "gazebo": True, "unity": False, "real": False}
# Unity: set use_sim_time:=true once the MobilityBridge publishes /clock (OPEN Q5).


def _truthy(s):
    return str(s).lower() in ("1", "true", "yes", "on")


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in
         ("domain", "input", "include_sim", "demo", "publish_odom", "use_sim_time",
          "extra_params", "world", "gui", "keyboard_prefix", "start_endpoint", "ros_ip",
          "unity_control")}
    domain = a["domain"]
    if domain not in ADAPTERS:
        raise RuntimeError(f"domain must be one of {list(ADAPTERS)}, got '{domain}'")
    share = get_package_share_directory("mobility_bringup")
    cfg = lambda f: os.path.join(share, "config", f)  # noqa: E731
    sim_time = SIM_TIME[domain] if a["use_sim_time"] == "auto" else _truthy(a["use_sim_time"])
    params = [cfg("geometry.yaml"), cfg("common.yaml"), cfg(f"{domain}.yaml")]
    if a["extra_params"]:
        params.append(a["extra_params"])
    p = params + [{"use_sim_time": sim_time}]

    acts = [LogInfo(msg=f"[mobility] domain={domain} input={a['input']} "
                        f"use_sim_time={sim_time}")]

    # -- simulator / bridge ---------------------------------------------------------
    if domain == "gazebo" and _truthy(a["include_sim"]):
        try:
            rb = get_package_share_directory("rover_bringup")
        except PackageNotFoundError:
            raise RuntimeError("include_sim:=true needs rover_bringup: source "
                               "ROS2_Autonomous_2026_27/install/setup.zsh first")
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rb, "launch", "rover_sim.launch.py")),
            launch_arguments={"world": a["world"], "gui": a["gui"]}.items()))
    if domain == "unity" and _truthy(a["start_endpoint"]):
        try:
            get_package_share_directory("ros_tcp_endpoint")
        except PackageNotFoundError:
            raise RuntimeError("domain:=unity needs ros_tcp_endpoint: source "
                               "~/ROS_unity/ros_ws/install/setup.zsh first "
                               "(or start_endpoint:=false if it runs elsewhere)")
        acts.append(Node(package="ros_tcp_endpoint", executable="default_server_endpoint",
                         name="ros_tcp_endpoint", output="screen",
                         parameters=[{"ROS_IP": a["ros_ip"], "ROS_TCP_PORT": 10000}]))

    # Unity twist mode: Unity's own servos drive from /cmd_vel; no PID stack runs here.
    twist_only = domain == "unity" and a["unity_control"] == "twist"
    if a["unity_control"] not in ("effort", "twist"):
        raise RuntimeError("unity_control must be effort or twist")
    if twist_only and _truthy(a["demo"]):
        raise RuntimeError("demo:=true checks the PIDs; use unity_control:=effort")

    if not twist_only:
        # -- domain adapter (the only domain-specific node) ---------------------------
        adapter = ADAPTERS[domain]
        acts.append(Node(package="mobility_hw", executable=adapter, name=adapter,
                         output="screen", parameters=p))

        # -- identical core -----------------------------------------------------------
        acts.append(Node(package="mobility_ros", executable="swerve_kinematics",
                         name="swerve_kinematics", output="screen", parameters=p))
        acts.append(Node(package="mobility_ros", executable="wheel_controller",
                         name="wheel_controller", output="screen", parameters=p))
    # The demo's checks read wheel odometry, so it always starts with demo:=true.
    if _truthy(a["publish_odom"]) or _truthy(a["demo"]):
        acts.append(Node(package="mobility_ros", executable="wheel_odometry",
                         name="wheel_odometry", output="screen", parameters=p))

    # -- operator input ---------------------------------------------------------------
    tp = params  # teleop runs on wall-clock in every domain (use_sim_time false in yaml)
    if a["input"] == "joy":
        acts.append(Node(package="joy", executable="joy_node", name="joy_node",
                         parameters=tp))
        acts.append(Node(package="mobility_ros", executable="teleop_joy",
                         name="teleop_joy", output="screen", parameters=tp))
    elif a["input"] == "keyboard":
        # Key window: real key press/release, several keys at once (W+A arcs).
        acts.append(Node(package="mobility_ros", executable="teleop_key_window",
                         name="teleop_key_window", output="screen", parameters=tp))
    elif a["input"] == "keyboard_terminal":
        acts.append(Node(package="mobility_ros", executable="teleop_keyboard",
                         name="teleop_keyboard", output="screen", parameters=tp,
                         prefix=a["keyboard_prefix"] or None))
    elif a["input"] != "none":
        raise RuntimeError("input must be keyboard, keyboard_terminal, joy or none")

    if _truthy(a["demo"]):
        demo = Node(package="mobility_ros", executable="scripted_drive",
                    name="scripted_drive", output="screen", parameters=p)
        acts.append(demo)
        acts.append(RegisterEventHandler(OnProcessExit(
            target_action=demo, on_exit=[EmitEvent(event=Shutdown(reason="demo done"))])))
    return acts


def generate_launch_description():
    args = [
        ("domain", "mock", "mock | gazebo | unity | real"),
        ("input", "none", "keyboard (key window) | keyboard_terminal | joy | none "
                          "(none = Nav2 / scripts drive /cmd_vel)"),
        ("include_sim", "false", "gazebo: also launch rover_bringup/rover_sim.launch.py"),
        ("world", "empty_world.sdf", "gazebo world when include_sim:=true"),
        ("gui", "true", "gazebo gui when include_sim:=true"),
        ("demo", "false", "run the scripted acceptance drive, then shut down"),
        ("publish_odom", "false", "run wheel_odometry (always on with demo:=true); "
                                  "localisation normally uses rover_ekf instead"),
        ("use_sim_time", "auto", "auto = per domain (gazebo true, others false)"),
        ("extra_params", "", "optional extra yaml loaded last (local overrides)"),
        ("keyboard_prefix", "gnome-terminal --wait --",
         "terminal for input:=keyboard_terminal ('' = this terminal, no stdin under launch)"),
        ("start_endpoint", "true", "unity: start ros_tcp_endpoint"),
        ("ros_ip", "127.0.0.1", "unity: ros_tcp_endpoint bind address"),
        ("unity_control", "effort", "unity: effort = our 8 PIDs drive (MobilityBridge "
                                    "Effort mode) | twist = only /cmd_vel, Unity's servos drive"),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=setup)])
