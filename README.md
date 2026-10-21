0. One-time setup: dependencies

# Gazebo/ROS bridge, ros2_control and the other tools the package needs
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro ros-jazzy-rviz2 \
  python3-colcon-common-extensions python3-rosdep python3-tk python3-pytest

# Resolve anything else listed in package.xml
sudo rosdep init 2>/dev/null; rosdep update
cd ~/twist_gazebo
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y



1. Clean build

Your git status shows that build/ and install/ are half-deleted and out of date, so start from scratch:

cd ~/twist_gazebo
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

--symlink-install lets Python edits take effect without a rebuild. You still need to rebuild after changing setup.py or adding new launch, config or URDF files.

Check that the build worked:
ros2 pkg executables twist_2_0_description
# should list: cmd_vel_gait, diagnose_gait, stand_pose, teleop_keyboard,
#              teleop_ui, walk_gait, walk_gait_cartesian


2. Run the tests (kinematics and gait)

cd ~/twist_gazebo
python3 -m pytest src/twist_2_0_description/test -v
# or through colcon:
colcon test --packages-select twist_2_0_description && colcon test-result --verbose

3. Check the model in RViz (no physics)

source ~/twist_gazebo/install/setup.bash
ros2 launch twist_2_0_description display.launch.py

Move the joint sliders to confirm the URDF, meshes and joint limits look right, then press Ctrl+C.


4. Gazebo and controllers only

# Terminal 1
source ~/twist_gazebo/install/setup.bash
ros2 launch twist_2_0_description gazebo.launch.py

# Terminal 2: check the controllers came up
source ~/twist_gazebo/install/setup.bash
ros2 control list_controllers
# both of these should be "active":
#   joint_state_broadcaster, leg_controller

# Command the stand pose
ros2 run twist_2_0_description stand_pose

If the controllers don't load, spawn them by hand:

ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner leg_controller


5. Autonomous walk demo (one terminal)

source ~/twist_gazebo/install/setup.bash
ros2 launch twist_2_0_description gazebo_walk_cartesian.launch.py

The sequence is a 5 s startup delay, then 3 s moving to the stand pose, then a continuous crawl gait (FL → RR → RL → FR). There is also a legacy joint-space version you can compare against: gazebo_walk.launch.py.



6. Full interactive pipeline: cmd_vel gait plus teleop

# Terminal 1: Gazebo, robot, controllers and the cmd_vel_gait node
source ~/twist_gazebo/install/setup.bash
ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py
# wait until you see:  [cmd_vel_gait] cmd_vel gait ready (mode=omni).

# Terminal 2: keyboard teleop (keep this terminal focused)
source ~/twist_gazebo/install/setup.bash
ros2 launch twist_2_0_description teleop_keyboard.launch.py







