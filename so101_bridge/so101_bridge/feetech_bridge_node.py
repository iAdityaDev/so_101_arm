#!/usr/bin/env python3
"""
feetech_bridge_node.py

Bridges lerobot's SO101Follower (which wraps FeetechMotorsBus) to plain ROS2
topics, so a ros2_control hardware interface (C++, non-Python) can drive the
real SO-101 arm without re-implementing motor communication or the tick<->degree
calibration math -- both are reused as-is from your installed lerobot package.

This node OWNS the serial connection to the arm. Only one process may hold the
port open at a time -- do not run lerobot-calibrate, lerobot-teleoperate, or any
other lerobot script against the same --robot.port while this node is running.

Topics:
    Subscribes  <commands_topic>  sensor_msgs/JointState   position in RADIANS
    Publishes   <states_topic>    sensor_msgs/JointState   position in RADIANS

Messages are matched by joint `name`, not by array index, so the ordering used
by your URDF/ros2_control does not need to match the ordering lerobot uses
internally.

Your URDF joint names very likely do NOT match lerobot's internal motor names
(e.g. your URDF might use "Shoulder_Rotation" while lerobot calls that motor
"shoulder_pan"). Rather than rename anything in your URDF/MoveIt/SRDF, set
`joint_name_map` below to translate between the two -- the default already
matches the standard so_arm_100 5-DOF naming convention.

Parameters:
    port                     (string)  default: /dev/ttyACM0
    robot_id                 (string)  default: my_awesome_follower_arm
    commands_topic           (string)  default: /so101/hardware_commands
    states_topic             (string)  default: /so101/hardware_states
    publish_rate_hz           (double)  default: 100.0
    max_relative_target_deg  (double)  default: 0.0  (0.0 = disabled)
        If > 0, caps how far the arm is allowed to move per command relative to
        its CURRENT position, in degrees. This is a per-cycle safety clamp --
        strongly recommended while validating the pipeline for the first time,
        e.g. start with 3-5 degrees so a bad MoveIt trajectory can't snap the
        arm to a wrong position in one step.
    joint_name_map           (string array)
        Each entry is "lerobot_motor_name:urdf_joint_name". Any lerobot motor
        not listed here is assumed to already share its name with the URDF.
        Default maps the 5 motors you currently have to the so_arm_100 naming
        convention (no gripper entry yet, since motor 6 isn't installed):
            shoulder_pan  -> Shoulder_Rotation
            shoulder_lift -> Shoulder_Pitch
            elbow_flex    -> Elbow
            wrist_flex    -> Wrist_Pitch
            wrist_roll    -> Wrist_Roll
"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower

# ============================================================
# EDIT THESE DIRECTLY, then rebuild (colcon build) to apply.
# No ROS2 parameters needed for these two -- just change the number and rebuild.
# ============================================================

# Feetech "Acceleration" register (0-254). Lower = gentler ramp up/down.
# lerobot's own default is 254 (fast). Leave at 254 unless you also want a
# slower ramp-up/ramp-down, separate from the top speed cap below.
SERVO_ACCELERATION = 254

# Feetech "Goal_Velocity" register -- a hard speed cap enforced by the servo
# itself, regardless of where a command tells it to go. 0 = unlimited (leaves
# whatever's currently set on the motor untouched). Start LOW (e.g. 30) and
# increase gradually while watching the real arm move -- see the safety
# calibration steps from earlier in this chat.
SERVO_GOAL_VELOCITY = 200

# ============================================================


class FeetechBridgeNode(Node):
    def __init__(self):
        super().__init__("feetech_bridge_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("robot_id", "my_awesome_follower_arm")
        self.declare_parameter("commands_topic", "/so101/hardware_commands")
        self.declare_parameter("states_topic", "/so101/hardware_states")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("max_relative_target_deg", 0.0)
        self.declare_parameter(
            "joint_name_map",
            [
                "shoulder_pan:Shoulder_Rotation",
                "shoulder_lift:Shoulder_Pitch",
                "elbow_flex:Elbow",
                "wrist_flex:Wrist_Pitch",
                "wrist_roll:Wrist_Roll",
            ],
        )

        port = self.get_parameter("port").get_parameter_value().string_value
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        commands_topic = self.get_parameter("commands_topic").get_parameter_value().string_value
        states_topic = self.get_parameter("states_topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        max_rel_deg = self.get_parameter("max_relative_target_deg").get_parameter_value().double_value
        raw_joint_map = self.get_parameter("joint_name_map").get_parameter_value().string_array_value

        self._lerobot_to_urdf: dict[str, str] = {}
        for entry in raw_joint_map:
            lerobot_name, sep, urdf_name = entry.partition(":")
            if not sep or not urdf_name:
                self.get_logger().warn(
                    f"Malformed joint_name_map entry '{entry}' (expected "
                    f"'lerobot_name:urdf_name'), ignoring it."
                )
                continue
            self._lerobot_to_urdf[lerobot_name] = urdf_name
        self._urdf_to_lerobot = {v: k for k, v in self._lerobot_to_urdf.items()}

        max_relative_target = max_rel_deg if max_rel_deg > 0.0 else None
        if max_relative_target is None:
            self.get_logger().warn(
                "max_relative_target_deg=0.0 (disabled). Commands will be sent to the "
                "arm with NO per-cycle safety clamp. Consider setting this to a small "
                "value (e.g. 5.0) until you've fully validated the pipeline."
            )

        cfg = SO101FollowerConfig(
            port=port,
            id=robot_id,
            use_degrees=True,
            disable_torque_on_disconnect=True,
            max_relative_target=max_relative_target,
        )
        self._robot = SO101Follower(cfg)

        self.get_logger().info(f"Connecting to SO-101 follower on '{port}' (id='{robot_id}')...")
        # calibrate=False requires a calibration file for this id to already exist
        # on disk. It never blocks on input() as long as is_calibrated is True,
        # which is the state you're in after a successful lerobot-calibrate run.
        self._robot.connect(calibrate=False)
        motor_names = list(self._robot.bus.motors.keys())
        self.get_logger().info(f"Connected. Motors under control: {motor_names}")

        # Acceleration: lerobot's own configure() already sets this to 254 (near-max)
        # for every motor -- only re-write it if SERVO_ACCELERATION above is different.
        if SERVO_ACCELERATION != 254:
            for motor in motor_names:
                self._robot.bus.write("Acceleration", motor, SERVO_ACCELERATION)
            self.get_logger().info(f"Set Acceleration={SERVO_ACCELERATION} on all motors.")

        # Goal_Velocity: lerobot never writes this for arm joints, so it's whatever
        # was last left on the motor (0 = "leave it alone"). Only applied if
        # SERVO_GOAL_VELOCITY above is > 0.
        if SERVO_GOAL_VELOCITY > 0:
            for motor in motor_names:
                self._robot.bus.write("Goal_Velocity", motor, SERVO_GOAL_VELOCITY)
            self.get_logger().info(
                f"Set Goal_Velocity={SERVO_GOAL_VELOCITY} on all motors (lower = slower)."
            )

        self._lock = threading.Lock()

        self._commands_sub = self.create_subscription(
            JointState, commands_topic, self._on_commands, 10
        )
        self._states_pub = self.create_publisher(JointState, states_topic, 10)

        period_s = 1.0 / rate_hz
        self._timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            f"Bridge ready. Commands: '{commands_topic}' -> Feetech bus -> States: '{states_topic}'"
        )

    def _on_commands(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self.get_logger().warn(
                "Received JointState with mismatched name/position array lengths, ignoring."
            )
            return

        known_motors = self._robot.bus.motors.keys()
        action = {}
        for urdf_name, pos_rad in zip(msg.name, msg.position):
            lerobot_name = self._urdf_to_lerobot.get(urdf_name, urdf_name)
            if lerobot_name not in known_motors:
                self.get_logger().warn(
                    f"Commanded joint '{urdf_name}' (mapped to '{lerobot_name}') is not "
                    f"a known motor {list(known_motors)}, skipping it."
                )
                continue
            action[f"{lerobot_name}.pos"] = math.degrees(pos_rad)

        if not action:
            return

        with self._lock:
            try:
                self._robot.send_action(action)
            except Exception as exc:  # noqa: BLE001 - surface any hardware error, keep node alive
                self.get_logger().error(f"send_action failed: {exc}")

    def _on_timer(self) -> None:
        with self._lock:
            try:
                obs = self._robot.get_observation()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"get_observation failed: {exc}")
                return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for key, val_deg in obs.items():
            if not key.endswith(".pos"):
                continue
            lerobot_name = key.removesuffix(".pos")
            urdf_name = self._lerobot_to_urdf.get(lerobot_name, lerobot_name)
            msg.name.append(urdf_name)
            msg.position.append(math.radians(val_deg))

        self._states_pub.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info("Disconnecting from SO-101 follower (torque will be disabled)...")
        try:
            self._robot.disconnect()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error during disconnect: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = FeetechBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()