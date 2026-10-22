# Twist 2.0 Quadruped: Gazebo Simulation and Walking

This is the ROS 2 Jazzy + Gazebo Harmonic package (`twist_2_0_description`) for the Twist 2.0
quadruped (12 × Feetech STS3215). It contains:

- the URDF, meshes and `ros2_control` setup
- a trot gait driven by `/cmd_vel`
- keyboard teleop
- an automated end-to-end walking test

Details on the robot (frames, joint limits, masses) are in [docs/ROBOT.md](docs/ROBOT.md).

## How the walking works

- **Kinematics** (`kinematics.py`): the leg geometry is copied directly from the URDF. Inverse
  kinematics uses damped least squares and stays within the URDF's recommended joint limits.
- **Gait** (`gait.py`): the feet follow Cartesian trajectories.
  - Feet on the ground slide back under the body at exactly the commanded twist, so they don't slip.
  - Swing feet lift 2.5 cm and land so the next stride is centred on their home position.
  - Commands are limited in acceleration. Any command the legs or servos can't reach is scaled down.
  - When the command goes to zero, the robot finishes one cycle so every foot is home, then stands still.
  - There are two gaits: `trot` (default) and `crawl` (slow, 3 feet always down).
- **Node** (`cmd_vel_gait`): waits for `/joint_states` and stands up over 2 s, then follows
  `/cmd_vel` (`geometry_msgs/Twist`). It runs on sim time and streams joint targets to
  `leg_controller` (JointTrajectoryController).
- **Gazebo**:
  - `gz_ros2_control` drives all 12 joints in position mode. Its `position_proportional_gain` is
    set to 1.0 so the simulated servos track the gait without lag.
  - An OdometryPublisher plugin publishes the robot's true pose on `/odom`.
- **Speed limits**: 0.15 m/s forward, 0.12 m/s sideways, 0.8 rad/s turning.

Measured in Gazebo (`gait_test`): forward/back 0.097 m/s at a 0.10 m/s command, strafe 0.077 m/s
at 0.08, turning 0.595 rad/s at 0.60. Body tilt stays at 4° or less.

```
config/      ros2_controllers.yaml, walk_params.yaml (gait), teleop_params.yaml
launch/      gazebo.launch.py, gazebo_with_cmd_vel.launch.py, gazebo_walk.launch.py,
             teleop_keyboard.launch.py, teleop.launch.py, display.launch.py
twist_2_0_description/
             kinematics.py, gait.py, cmd_vel_gait.py, walk_gait.py, stand_pose.py,
             gait_test.py, teleop_keyboard.py, teleop_ui.py
urdf/        twist_2_0.urdf, twist_2_0_gazebo.urdf.xacro, twist_2_0_isaac.urdf
meshes/      visual + collision STLs
test/        unit tests (kinematics vs URDF, IK, gait reachability/limits/stopping)
```

## 1. Install dependencies (one time)

Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep python3-numpy python3-tk python3-pytest
```

## 2. Get the code and build

```bash
mkdir -p ~/twist_ws/src
cd ~/twist_ws/src
git clone https://github.com/VarunManikonda1/Twist.git twist_2_0_description

cd ~/twist_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional, picks up anything missing
colcon build
source install/setup.bash
```

Check the build:

```bash
ros2 pkg executables twist_2_0_description
# cmd_vel_gait  gait_test  stand_pose  teleop_keyboard  teleop_ui  walk_gait
```

> Every new terminal needs `source /opt/ros/jazzy/setup.bash && source ~/twist_ws/install/setup.bash`.

## 3. Unit tests

```bash
cd ~/twist_ws/src/twist_2_0_description
python3 -m pytest -q test/
```

## 4. Look at the model in RViz (optional, no physics)

```bash
ros2 launch twist_2_0_description display.launch.py
```

## 5. Run the full pipeline: Gazebo + gait + teleop

**Terminal 1: Gazebo, robot, controllers and gait node**

```bash
ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py
```

Wait for `Standing. Send geometry_msgs/Twist on /cmd_vel to walk.`

**Terminal 2: drive with `teleop_twist_keyboard`**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.6
```

Keep this terminal focused while you press keys.

| Key | Motion |
|---|---|
| `i` / `,` | forward / backward |
| `j` / `l` | turn left / right on the spot |
| `u` / `o` | forward arc left / right |
| `m` / `.` | backward arc |
| `k` | stop |
| `J` / `L` (Shift) | step sideways left / right |

One key press keeps the robot moving until you press `k`, because `cmd_timeout: 0.0` keeps the
last command. Speed keys `q`/`z` only matter below the gait's limits (0.15 m/s, 0.8 rad/s).

**Or use the package's own teleop.** It publishes continuously and has omni/ackermann modes:

```bash
ros2 run twist_2_0_description teleop_keyboard
# or open it in a new gnome-terminal window:
ros2 launch twist_2_0_description teleop_keyboard.launch.py
```

`i`/`,` forward/back, `j`/`l` strafe (omni) or turn (ackermann), `u`/`o` rotate, `space` stop,
`m` switch mode, `q` quit.

**Or publish `/cmd_vel` directly:**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.6}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"      # stop
```

## 6. Automated end-to-end test

With the Terminal 1 launch running (with or without the GUI), in another terminal:

```bash
ros2 run twist_2_0_description gait_test --ros-args -p use_sim_time:=true
```

The test drives these moves:

- forward, backward
- strafe left, strafe right
- turn CCW, turn CW
- an arc, a fast walk (0.15 m/s) and a diagonal
- stops between moves

It measures each move from `/odom` and prints a PASS/FAIL table with `OVERALL: PASS` at the end.
It exits non-zero on failure. It takes about 90 s.

Headless (no Gazebo window, e.g. CI or SSH):

```bash
ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py gz_args:="-r -s empty.sdf"
```

## 7. Autonomous walk (no teleop)

```bash
ros2 launch twist_2_0_description gazebo_walk.launch.py
# change it live:
ros2 param set /walk_gait walk_vx 0.0
ros2 param set /walk_gait walk_wz 0.5
```

## 8. Monitor and tune

```bash
ros2 control list_controllers                   # joint_state_broadcaster + leg_controller active
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /joint_states --field position
ros2 param list /cmd_vel_gait

ros2 param set /cmd_vel_gait step_height 0.03     # higher foot lift
ros2 param set /cmd_vel_gait cycle_period 0.5     # faster stepping
ros2 param set /cmd_vel_gait body_height 0.13     # lower body
ros2 param set /cmd_vel_gait gait_style crawl     # slow, very stable (also set cycle_period 1.2)
ros2 param set /cmd_vel_gait cmd_timeout 0.5      # stop if /cmd_vel goes quiet for 0.5 s
```

Defaults are in `config/walk_params.yaml`. Rebuild after editing it.

## Troubleshooting

- **Robot doesn't move:** check that `cmd_vel_gait` printed `Standing.` and that
  `ros2 control list_controllers` shows both controllers active. Also make sure only one gait node
  is running (`ros2 node list`).
- **Keys do nothing:** the teleop terminal must have keyboard focus.
- **Old Gazebo still running / duplicate controllers:** run `pkill -f "gz sim"`, then launch again.
- **Stale build after pulling:** `rm -rf build install log && colcon build`.
