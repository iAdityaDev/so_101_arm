# SO-101 Real Hardware Bridge for MoveIt / ros2_control

Drives a real SO-ARM100/SO-101 Feetech-servo arm through MoveIt2, by bridging
`ros2_control` to [lerobot](https://github.com/huggingface/lerobot)'s
`SO101Follower` instead of talking to the Feetech serial protocol directly.
Swaps in as a drop-in replacement for a MuJoCo/mock hardware plugin — MoveIt,
the SRDF, and your controllers config don't need to change.

> **Current hardware status:** this arm is running with **2 motors only**
> (`Shoulder_Rotation` / `Shoulder_Pitch`) — the elbow, wrist, and gripper
> motors have been physically removed. See
> [Current hardware configuration](#current-hardware-configuration) below.

## Why a bridge instead of a native ros2_control serial driver

A native C++ `SystemInterface` that opens the Feetech serial connection
directly is the more common pattern (and worth doing eventually — see
[Future work](#future-work)). This project instead reuses lerobot's own,
already-tested `FeetechMotorsBus` and calibration math via a small Python
bridge node, trading a bit of IPC latency for zero risk of re-implementing
tick↔angle conversion incorrectly on first hardware contact.

## Architecture

```
MoveIt  ->  joint_trajectory_controller  ->  controller_manager
                                                    |
                                    SO101HardwareInterface (C++, so101_hardware)
                                       write(): publish JointState (radians)
                                       read():  cached JointState (radians)
                                                    |
                                 ROS2 topics (matched by joint name, not order)
                          /so101/hardware_commands, /so101/hardware_states
                                                    |
                               feetech_bridge_node (Python, so101_bridge)
                     renames joints, converts radians<->degrees, safety clamps
                                                    |
                            lerobot SO101Follower / FeetechMotorsBus
                       degrees <-> raw encoder ticks, via calibration JSON
                                                    |
                                    Feetech STS3215 servos (serial)
```

Two ROS2 packages:

| Package | Type | Role |
|---|---|---|
| `so101_hardware` | `ament_cmake` (C++) | `hardware_interface::SystemInterface` plugin. Never touches serial — only ROS2 topics. |
| `so101_bridge` | `ament_python` | Owns the serial connection via lerobot. Only process allowed to hold the port open. |

## Repository layout

```
.
├── so101_bridge/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/so101_bridge
│   └── so101_bridge/
│       └── feetech_bridge_node.py
├── so101_hardware/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── so101_hardware.xml
│   ├── include/so101_hardware/so101_hardware_interface.hpp
│   └── src/so101_hardware_interface.cpp
└── README.md
```

This repo does **not** contain the arm's URDF/xacro, MoveIt config,
`controllers.yaml`, or SRDF — those live in your existing
`so_arm_100_description` / `so_arm_100_moveit_config` packages, patched per
the [ros2_control integration](#ros2_control-integration-in-your-urdf)
section below.

---

## Prerequisites

- ROS2 (developed against Humble) with `ros2-control`, `ros2-controllers`,
  `pluginlib` installed (`sudo apt install ros-<distro>-ros2-control
  ros-<distro>-ros2-controllers ros-<distro>-pluginlib`).
- A completed `lerobot-calibrate` run for your arm's current motor
  configuration, with the resulting calibration JSON present at
  `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<robot_id>.json`.
- A local, patched clone of `lerobot` (see below) — this project relies on
  editing `lerobot`'s `so_follower.py` motor list to match your actual,
  physically-installed motor count.

## Environment setup

`rclpy` (compiled against system Python/ROS2's DDS libraries) and `lerobot`
(a large, actively-updated pip package) need to coexist in one Python
environment. A conda env cannot reliably do this — conda ships its own
Python build and shared libraries, which conflicts with `rclpy`'s compiled
linkage to the system's. Use a **venv with system-site-packages** instead:

```bash
sudo apt install -y python3-venv
python3 -m venv ~/ros2_lerobot_env --system-site-packages
source ~/ros2_lerobot_env/bin/activate
python3 -m pip install --upgrade pip setuptools wheel
```

Install your patched lerobot clone (see
[Local lerobot patches](#local-lerobot-patches-required) below) with the
`feetech` extra, which pulls in `feetech-servo-sdk` (imported as
`scservo_sdk`) — confirmed directly against lerobot's own `pyproject.toml`,
not bundled by default:

```bash
cd ~/lerobot
pip install -e '.[feetech]'
```

Verify:
```bash
python3 -c "import rclpy; import lerobot; import scservo_sdk; print('ALL OK')"
```

**If you hit `ImportError`/`ValueError: numpy.dtype size changed` or similar
binary-incompatibility errors** for `pandas`, `PIL`/`Pillow`, `scipy`, or
similar packages: `--system-site-packages` exposes Ubuntu's older
apt-installed copies of these, which can get imported instead of (or in
conflict with) the fresh versions your venv's `lerobot` install pulled in.
Fix per-package as encountered:
```bash
pip install --upgrade --force-reinstall <package_name>
```

**Every terminal used for building or running anything in this repo needs,
in this exact order, every time:**
```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_lerobot_env/bin/activate
```
No conda active, ever, in any terminal touching this project. `colcon build`
specifically must be run with the venv active — the interpreter active *at
build time* gets baked into each package's generated launcher script shebang
line, and rebuilding is the only fix if this is missed (`rm -rf
build/<pkg> install/<pkg> log && colcon build --packages-select <pkg>`).

## Local lerobot patches required

This project relies on editing lerobot's `so_follower.py` motor dictionary
to match your arm's actual, physically-installed motor count — the
calibration handshake fails outright for any motor listed in code but not
physically present.

Current state: only `shoulder_pan` (id 1) and `shoulder_lift` (id 2) should
remain uncommented; `elbow_flex`, `wrist_flex`, `wrist_roll`, and `gripper`
are all commented out.

After any change to this file, **recalibrate**:
```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=<your_robot_id>
```

## Build

```bash
mkdir -p ~/so101_moveit_ws/src && cd ~/so101_moveit_ws/src
# place so101_bridge/ and so101_hardware/ here, alongside your existing
# so_arm_100_description / so_arm_100_moveit_config packages
cd ~/so101_moveit_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select so101_bridge so101_hardware
source install/setup.bash
```

## ros2_control integration in your URDF

In your arm's `.ros2_control.xacro`, the real-hardware branch of the
`<hardware>` block:
```xml
<plugin>so101_hardware/SO101HardwareInterface</plugin>
<param name="commands_topic">/so101/hardware_commands</param>
<param name="states_topic">/so101/hardware_states</param>
```
No serial port, baud rate, or servo-speed params belong in the URDF at all —
those live entirely in the bridge node's launch parameters and the constants
described below.

Per-joint `<command_interface name="position">` min/max should match your
actual link URDF's `<limit lower=".." upper="..">` values, not a placeholder.

## Configuration

### `feetech_bridge_node.py` — ROS2 launch parameters

| Parameter | Default | Purpose |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial port to the arm |
| `robot_id` | `my_awesome_follower_arm` | Must match your `lerobot-calibrate` `--robot.id` |
| `commands_topic` / `states_topic` | `/so101/hardware_commands` / `/so101/hardware_states` | Must match the URDF's `<param>`s |
| `publish_rate_hz` | `100.0` | State-publishing timer rate |
| `max_relative_target_deg` | `0.0` (disabled) | Per-command safety clamp, in degrees, relative to current position. **Strongly recommended non-zero** (e.g. `5.0`) until fully validated. |
| `joint_name_map` | so_arm_100 naming convention | `"lerobot_name:urdf_name"` entries translating between lerobot's motor names and your URDF's joint names |

### `feetech_bridge_node.py` — hardcoded constants (edit file directly, then rebuild)

| Constant | Default | Purpose |
|---|---|---|
| `SERVO_ACCELERATION` | `150` | Feetech `Acceleration` register (0-254). Lower = gentler ramp up/down. lerobot's own default is 254. |
| `SERVO_GOAL_VELOCITY` | `250` | Feetech `Goal_Velocity` register — hard speed cap enforced by the servo itself, regardless of command source. `0` = leave unchanged. **This, not `max_relative_target_deg`, is what actually caps physical speed.** |
| `FAKE_JOINT_NAMES_URDF` | `["Elbow", "Wrist_Pitch", "Wrist_Roll", "Gripper"]` | Temporary: publishes static `0.0` for joints with no installed motor, so `on_activate()` doesn't hang waiting for them. See [Current hardware configuration](#current-hardware-configuration). |

Tuning `SERVO_GOAL_VELOCITY`: the exact steps/sec scale isn't reliably
documented by Feetech, so find a safe value empirically — start very low
(~30), double it and re-test repeatedly on a full-range motion, stop well
before any speed you're not comfortable being near, then back off one more
doubling step for margin.

---

## Launch sequence

Four terminals, all with the environment sourced per
[Environment setup](#environment-setup), started in this order, all left
running — closing any one of them (even briefly) breaks the chain and
requires restarting from that point.

**Terminal 1 — bridge node** (must be running before anything else attempts
to activate):
```bash
ros2 run so101_bridge feetech_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p robot_id:=<your_robot_id> \
  -p max_relative_target_deg:=5.0
```
Expect: `Connected. Motors under control: [...]` then `Bridge ready.`

**Terminal 2 — robot_state_publisher:**
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro /path/to/your_top_level.urdf.xacro)"
```

**Terminal 3 — controller_manager.** Note the **quoted** remap — an
unquoted `~/robot_description` gets expanded by bash itself into
`$HOME/robot_description` before ROS2 ever sees it, a silent failure mode
that produces no error and no warning:
```bash
ros2 run controller_manager ros2_control_node --ros-args \
  --remap '~/robot_description:=/robot_description' \
  --params-file /path/to/your/controllers.yaml
```
Expect: `Loading hardware...` → `Successful initialization` → `Successful
'configure'` → `Waiting for first state message from bridge node...` →
`SO101HardwareInterface activated.`

**Terminal 4 — controllers, then MoveIt:**
```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner <your_arm_controller_name>
ros2 launch <your_moveit_config_pkg> move_group.launch.py
```
Then RViz, plan a small motion, inspect the preview, execute.

## Validation order (do this in sequence on any fresh setup)

1. Bridge alone + move the arm by hand, confirm `/so101/hardware_states`
   reports correct direction per joint.
2. One manual command via `ros2 topic pub --once
   /so101/hardware_commands ...`, confirm correct small motion.
3. Full bring-up, RViz only (no MoveIt plan/execute yet) — confirm the
   displayed model matches the real arm's pose.
4. MoveIt plan+execute, with `max_relative_target_deg` still small.
5. Only then raise/disable the safety clamp for normal operation.

---

## Current hardware configuration

The arm currently has **2 of its original motors installed** —
`shoulder_pan`/`shoulder_lift` (`Shoulder_Rotation`/`Shoulder_Pitch` in the
URDF). `elbow_flex`, `wrist_flex`, `wrist_roll`, and the gripper motor have
all been physically removed.

**What's been done:**
- `so_follower.py` patched to only declare the 2 remaining motors.
- `feetech_bridge_node.py`'s `FAKE_JOINT_NAMES_URDF` publishes static `0.0`
  for the 4 removed joints, so hardware activation doesn't hang/fail waiting
  for motors that no longer exist.

**Known limitation of the current setup:** the 4 removed joints are still
declared `type="revolute"` in the link URDF with full collision geometry
attached. This has caused **self-collision false positives** (pink/magenta
highlighting in RViz) as the real `Shoulder_Pitch` joint moves while the
downstream, now-frozen-at-zero geometry sweeps along with it.

**Not yet done — required for a fully correct 2-motor setup:**
1. Convert `Elbow`, `Wrist_Pitch`, `Wrist_Roll`, `Gripper` from
   `type="revolute"` to `type="fixed"` in `so_arm_100_5dof_arm.urdf.xacro`
   (or remove their `<collision>` blocks entirely, since the hardware is
   physically gone, not just unpowered).
2. Trim `controllers.yaml`'s joint list down to the 2 real joints.
3. Trim the MoveIt SRDF's planning group down to the 2 real joints (re-run
   MoveIt Setup Assistant, or edit the `.srdf` directly).
4. Remove the corresponding 4 `<joint>` blocks from the `.ros2_control.xacro`
   file entirely (no C++ rebuild required — `SO101HardwareInterface` reads
   joint count from the URDF dynamically).

Until all four are done, `FAKE_JOINT_NAMES_URDF` is a working but incomplete
stopgap — the arm's real 2 DOF work correctly, but MoveIt's planning group
still nominally includes the 4 fake joints.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `pluginlib::LibraryLoadException ... does not exist` | Stale build, or workspace not sourced in this terminal | `rm -rf build/so101_hardware install/so101_hardware log && colcon build --packages-select so101_hardware`; confirm with `find install -path "*hardware_interface__pluginlib__plugin*"` |
| `ModuleNotFoundError: No module named 'lerobot'` when running via `ros2 run` | Package built with wrong Python (venv not active during `colcon build`) | Check `head -1 install/so101_bridge/lib/so101_bridge/feetech_bridge_node` — must point inside your venv, not `/usr/bin/python3`; rebuild with venv active |
| `RuntimeError: FeetechMotorsBus motor check failed ... Missing motor IDs` | `so_follower.py`'s motor list doesn't match physically-installed motors | Comment out the missing motor(s), recalibrate |
| `Bridge node never reported a position for joint 'X'` | URDF joint name not in `joint_name_map`, or that motor was removed without a `FAKE_JOINT_NAMES_URDF` entry | Add/check the mapping, or add the joint name to `FAKE_JOINT_NAMES_URDF` if intentionally absent |
| `name 'prefix' is not defined` during `xacro` expansion | `prefix` used in joint names but never declared as a macro param | Add `prefix:=^|''` to the macro's `params=` list |
| `controller_manager` hangs forever on `Subscribing to '~/robot_description'...` | Nothing publishing there, or (very commonly) an unquoted `~/robot_description:=...` remap got tilde-expanded by bash into a garbage path | Quote the remap: `--remap '~/robot_description:=/robot_description'`; confirm with `ros2 node info /controller_manager` |
| `ValueError: numpy.dtype size changed...` / similar for `pandas`/`PIL`/`scipy` | System apt package shadowing venv package via `--system-site-packages` | `pip install --upgrade --force-reinstall <package>` |
| Pink/magenta highlighting in RViz | Self-collision — check whether it's a real collision, or the [current hardware configuration](#current-hardware-configuration) issue with frozen downstream geometry |

## Safety notes

- `max_relative_target_deg` (bridge parameter) caps how far a single command
  can move the arm — protects against a *wrong destination*.
- `SERVO_GOAL_VELOCITY` (bridge constant) caps physical speed, enforced by
  the servo itself — protects against *dangerous speed*, independent of
  command correctness. Keep both active while testing; they protect against
  different failure modes.
- Only one process may hold the serial port open at a time — never run
  `lerobot-calibrate`/`lerobot-teleoperate` while `feetech_bridge_node` is
  running against the same port.

## Future work

- Native C++ Feetech serial driver (bypassing the Python bridge/IPC hop) for
  lower latency, once the topic-based pipeline above is fully trusted.
- Recalibrate and restore `elbow_flex`/`wrist_flex`/`wrist_roll`/gripper if
  those motors are reinstalled — reverse the `so_follower.py` patch, remove
  the corresponding `FAKE_JOINT_NAMES_URDF` entries, and complete the
  fixed-joint/collision/controllers.yaml/SRDF cleanup above regardless of
  which direction the motor count changes.
- Runtime (per-command, not just at-startup) servo speed control, if a use
  case beyond a fixed safety cap emerges.





# so_101_arm



https://github.com/user-attachments/assets/d9211cfa-1904-4dcd-8ea7-1023dfa931cb

https://github.com/user-attachments/assets/4afae117-0115-45f9-b2b0-257acea641ea

