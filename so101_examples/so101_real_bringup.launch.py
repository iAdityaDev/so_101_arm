"""
Example launch file for driving the REAL SO-101 arm through MoveIt.

Startup order matters here: the feetech_bridge_node must open the serial
port and start publishing joint states BEFORE the SO101HardwareInterface
activates, because on_activate() blocks (up to 5s) waiting for the first
state message. TimerAction below gives the bridge a head start; adjust the
delay if your machine needs more/less time to open the port.

This assumes you already have your own robot_description / MoveIt launch
files from the MuJoCo setup -- copy the relevant includes from those here,
replacing only the ros2_control xacro block (see so101_real.ros2_control.xacro)
and the hardware-specific launch args below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyACM0", description="Serial port for the SO-101 follower arm"
    )
    robot_id_arg = DeclareLaunchArgument(
        "robot_id", default_value="my_awesome_follower_arm",
        description="Must match the --robot.id you used with lerobot-calibrate",
    )
    max_relative_target_arg = DeclareLaunchArgument(
        "max_relative_target_deg", default_value="5.0",
        description="Per-cycle safety clamp in degrees. 0.0 disables it. "
                    "Start small (e.g. 5.0) until the pipeline is fully validated.",
    )

    bridge_node = Node(
        package="so101_bridge",
        executable="feetech_bridge_node",
        name="feetech_bridge_node",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "robot_id": LaunchConfiguration("robot_id"),
            "max_relative_target_deg": LaunchConfiguration("max_relative_target_deg"),
        }],
    )

    # Give the bridge node time to open the port, connect, and start
    # publishing states before controller_manager tries to activate the
    # hardware interface. Replace `controller_manager_launch` below with
    # your actual existing MoveIt/ros2_control bringup (the one that
    # currently loads the MuJoCo plugin) -- just swap its ros2_control
    # xacro include for so101_real.ros2_control.xacro.
    #
    # controller_manager_launch = IncludeLaunchDescription(...)
    #
    # delayed_controller_manager = TimerAction(
    #     period=3.0,
    #     actions=[controller_manager_launch],
    # )

    return LaunchDescription([
        port_arg,
        robot_id_arg,
        max_relative_target_arg,
        bridge_node,
        # delayed_controller_manager,
    ])
