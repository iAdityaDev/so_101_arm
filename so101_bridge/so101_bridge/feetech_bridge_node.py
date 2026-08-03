#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower


SERVO_ACCELERATION = 150
SERVO_GOAL_VELOCITY = 250
FAKE_JOINT_NAMES_URDF = ["Elbow", "Wrist_Pitch", "Wrist_Roll","Gripper"]

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
            pass

        cfg = SO101FollowerConfig(
            port=port,
            id=robot_id,
            use_degrees=True,
            disable_torque_on_disconnect=True,
            max_relative_target=max_relative_target,
        )
        self._robot = SO101Follower(cfg)

        self.get_logger().info(f"Connecting to SO-101 follower on '{port}' (id='{robot_id}')...")
        self._robot.connect(calibrate=False)
        motor_names = list(self._robot.bus.motors.keys())
        self.get_logger().info(f"Connected. Motors under control: {motor_names}")


        if SERVO_ACCELERATION != 254:
            for motor in motor_names:
                self._robot.bus.write("Acceleration", motor, SERVO_ACCELERATION)
            self.get_logger().info(f"Set Acceleration={SERVO_ACCELERATION} on all motors.")

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
            except Exception as exc:
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

####################################################################$
        for fake_name in FAKE_JOINT_NAMES_URDF:
            msg.name.append(fake_name)
            msg.position.append(0.0)
###############################################################

        self._states_pub.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info("Disconnecting SO-101")
        try:
            self._robot.disconnect()
        except Exception as exc: 
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