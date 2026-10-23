# Twist 2.0 Quadruped (twist_walker)

ROS 2 Jazzy + Gazebo Harmonic simulation of the Twist 2.0 quadruped, with a
URDF-driven inverse-kinematics trot gait that you drive from the keyboard.

## Pipeline

```
teleop_twist_keyboard ──/cmd_vel──► gait_node ──/leg_controller/commands──► leg_controller ──► Gazebo
                                       ▲  (smooth cmd → trot planner → IK)       (ros2_control)      │
                                       └──── /robot_description, /joint_states ◄─────────────────────┘
```

When you launch `sim.launch.py`, it starts everything in this order:

1. Gazebo Harmonic (`empty.sdf`) and `robot_state_publisher` (the xacro is expanded, with high friction added to the feet)
2. The `/clock` bridge (Gazebo → ROS)
3. Spawn the robot `twist_2_0`
4. `joint_state_broadcaster`
5. `leg_controller` (position control of all 12 joints)
6. `gait_node`, which blends the robot from its spawn pose into a stand over 2 s and then trots according to `/cmd_vel`

## Requirements

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Python | 3.12 (system) + NumPy |

## 1. Install ROS 2 Jazzy

Skip this step if `/opt/ros/jazzy` already exists.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Install the project dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-position-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  python3-numpy \
  python3-colcon-common-extensions \
  python3-rosdep
```

`ros-jazzy-ros-gz` pulls in Gazebo Harmonic, so you don't need to install Gazebo separately.

## 3. Create the workspace and clone

```bash
mkdir -p ~/twist_ws/src
cd ~/twist_ws/src
git clone https://github.com/VarunManikonda1/Twist.git twist_walker
```

## 4. Resolve any remaining dependencies with rosdep

```bash
sudo rosdep init        # only the first time on this machine (it's fine if it says it already exists)
rosdep update
cd ~/twist_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build

```bash
cd ~/twist_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To source the workspace automatically in every new terminal:

```bash
echo "source ~/twist_ws/install/setup.bash" >> ~/.bashrc
```

## 6. Run

### Terminal 1: simulation, controllers and gait

```bash
source ~/twist_ws/install/setup.bash
ros2 launch twist_walker sim.launch.py
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `gait` | `true` | Start `gait_node`. Set it to `false` to spawn the robot on its own. |
| `spawn_z` | `0.12` | Spawn height in metres |

```bash
ros2 launch twist_walker sim.launch.py gait:=false spawn_z:=0.15
```

### Terminal 2: drive with the keyboard

```bash
source ~/twist_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep this terminal focused while you drive. `i` moves forward, `,` moves back, `j`/`l` turn, `J`/`L` (Shift) strafe and `k` stops.
Use `z` to lower the speed. The gait clamps commands to 0.12 m/s forward, 0.08 m/s sideways and 0.6 rad/s turning.

Instead of the keyboard, you can send a single command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}"
```

If no `/cmd_vel` message arrives for 0.5 s, the robot stops in place.

### Optional: inspect the URDF in RViz

```bash
ros2 launch twist_walker display.launch.py
```

In RViz, set **Fixed Frame** to `base_link`, then **Add → RobotModel** and set **Description Topic** to `/robot_description`. Use the slider window to move each joint.

## 7. Record motor angles for the hardware

`record_walk` drives the robot by itself (stand 3 s, forward 5 s, stop, backward 5 s, stop) and saves
the angle of every Feetech STS3215 servo at 50 Hz. Start the simulation first (section 6, Terminal 1),
wait until the robot stands, then in a second terminal:

```bash
source ~/twist_ws/install/setup.bash
cd ~/twist_ws
ros2 run twist_walker record_walk --ros-args \
  --params-file install/twist_walker/share/twist_walker/config/motor_ids.yaml
```

Don't send `/cmd_vel` (teleop) while it runs. It stops by itself and writes to `~/motor_recordings/`:

| File | Contents |
|---|---|
| `walk_<time>_forward_deg.csv` | Forward part only: `time_s, ID1..ID12` in servo degrees (0-360, 180 = middle) |
| `walk_<time>_backward_deg.csv` | Backward part only, same format |
| `walk_<time>_deg.csv` | Whole run in servo degrees, with a `phase` column |
| `walk_<time>_ticks.csv` | Whole run in STS3215 steps (0-4095, 2048 = middle), with a `phase` column |
| `walk_<time>_ids.txt` | Which ID is which joint, and the direction/offset used |

Servo IDs:

| Leg | Hip yaw | Hip pitch | Knee |
|---|---|---|---|
| Front left (FL) | 1 | 2 | 3 |
| Front right (FR) | 4 | 5 | 6 |
| Rear left (RL) | 7 | 8 | 9 |
| Rear right (RR) | 10 | 11 | 12 |

View the newest recording:

```bash
column -s, -t < $(ls -t ~/motor_recordings/*_forward_deg.csv | head -1) | less
column -s, -t < $(ls -t ~/motor_recordings/*_backward_deg.csv | head -1) | less
cat $(ls -t ~/motor_recordings/*_ids.txt | head -1)
```

Conversion: `ticks = 2048 + direction * angle_rad * 4096/(2*pi) + offset`, `servo deg = ticks * 360/4096`.
IDs, directions, offsets and the walk timing are set in `config/motor_ids.yaml`.

**Before playing a recording on the robot:** calibrate it. Put the real robot in the URDF zero pose
(all sliders at 0 in `display.launch.py`), read each servo, and set `offset = reading - 2048`. If a servo
turns the opposite way to the sim joint, set its `direction` to `-1`. Rebuild and record again, then test
with the robot held in the air, moving slowly to the first row before starting playback.

To watch all 12 motors live (command, position, velocity, effort) while driving with teleop:

```bash
ros2 run twist_walker motor_logger      # also saves ~/motor_log_<time>.csv
```

## 8. Useful checks

```bash
ros2 control list_controllers                 # joint_state_broadcaster + leg_controller should be "active"
ros2 topic echo /joint_states --once
ros2 topic echo /leg_controller/commands --once
ros2 topic hz /leg_controller/commands        # ~50 Hz
gz model -m twist_2_0 -p                      # robot position in Gazebo (x grows when walking forward)
```

## Tuning

Gait parameters are in `config/gait.yaml`. With `--symlink-install` you only need to edit the file and relaunch, without rebuilding.

| Parameter | Default | Effect |
|---|---|---|
| `stand_height` | 0.12 m | How far the feet sit below `base_link` |
| `stance_out` | 0.095 m | How far each foot sits outward from its hip |
| `period` | 0.8 s | Length of one full trot cycle |
| `step_height` | 0.02 m | How high each foot lifts during a step |
| `max_step` | 0.05 m | Maximum stride per half-cycle |
| `max_vx` / `max_vy` / `max_wz` | 0.12 / 0.08 / 0.6 | Velocity limits |
| `cmd_smoothing` | 0.3 s | Low-pass filter on `/cmd_vel` |
| `rate` | 50 Hz | How often joint targets are published |

The controller setup (joint order and controller types) is in `config/controllers.yaml`. The joint order there must match `joints` in `gait.yaml`.

## Package layout

```
twist_walker/
├── config/
│   ├── controllers.yaml       # ros2_control: joint_state_broadcaster + leg_controller
│   ├── gait.yaml              # gait_node parameters
│   └── motor_ids.yaml         # record_walk: servo IDs, direction, offset, walk timing
├── launch/
│   ├── sim.launch.py          # Gazebo + spawn + controllers + gait
│   └── display.launch.py      # RViz + joint sliders
├── meshes/{visual,collision}/ # STL meshes
├── urdf/
│   ├── twist_2_0.urdf         # original CAD export (links, joints, meshes)
│   └── twist_walker.urdf.xacro# adds ros2_control + gz_ros2_control plugin
├── twist_walker/
│   ├── gait_node.py           # ROS node: /cmd_vel → gait → IK → commands
│   ├── gait.py                # trot foot-trajectory planner
│   ├── kinematics.py          # leg chains + IK built from the URDF
│   ├── record_walk.py         # drive forward/backward and save servo angles to CSV
│   └── motor_logger.py        # live table + CSV of all joint commands and states
├── package.xml
└── setup.py
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Package 'twist_walker' not found` | Run `source ~/twist_ws/install/setup.bash` in that terminal. |
| The robot appears with no meshes, or Gazebo can't find the meshes | Rebuild, then re-source. The launch file sets `GZ_SIM_RESOURCE_PATH` from the installed share directory. |
| The controllers never become active | Check that `ros-jazzy-gz-ros2-control` is installed, then kill old Gazebo processes with `pkill -f "gz sim"` and relaunch. |
| The robot doesn't move when you press keys | The teleop terminal must have focus. Check that `ros2 topic echo /cmd_vel` shows messages. |
| A warning says `stand pose not reachable` | Lower `stand_height` or `stance_out` in `gait.yaml`. |
| Gazebo is slow or black on a VM | Try `export LIBGL_ALWAYS_SOFTWARE=1` before launching. |

## License

MIT
