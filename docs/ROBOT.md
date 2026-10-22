# twist_2_0_description

URDF of the **Twist 2.0** quadruped (12 × Feetech STS3215), generated from `../Twist-2.0.iam` by
`../scripts/export_urdf.py`. Regenerate it after any CAD change; don't hand-edit it.
(This README is copied in from `../scripts/package_README.md` on every export, so edit it there.)

## Files

| Path | Use |
|---|---|
| `urdf/twist_2_0.urdf` | Main URDF, `package://twist_2_0_description/...` mesh paths (ROS 2, RViz, Gazebo) |
| `urdf/twist_2_0_isaac.urdf` | Same robot with relative mesh paths (`../meshes/...`) and the recommended range as joint limits. Import directly in **Isaac Sim** |
| `urdf/twist_2_0_gazebo.urdf.xacro` | Main URDF + `ros2_control` (gz_ros2_control, position command on all 12 joints, clamped to the recommended range) |
| `meshes/visual/` | Visual STL per link and material (aluminium, foot, board, standoffs, cable plugs), plus one shared `sts3215.stl` |
| `meshes/collision/` | One convex hull per part (each link also has a sphere at the foot tip) |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster` + `leg_controller` (JointTrajectoryController) |
| `launch/display.launch.py` | RViz + joint sliders |
| `launch/gazebo.launch.py` | gz-sim (Harmonic) + spawn + controllers |

## Robot

- **Frames:** `base_link` at the body center, at hip-yaw servo height. **X forward = the head**, marked by the U-shaped cut in the top plate. Y left, Z up. (The ESP32 USB/RJ45 ports face the robot's left side.)
- **Leg names:** front/back and left/right are measured from the head, the same as in the Inventor model (`LEG_FL`, `STS3215_HipYaw_FL`, ...). `../reference/Twist-2.0_orientation.png` shows the labels.
- **Links:** `base_link` → `{FL,FR,RL,RR}_hip` → `_thigh` → `_shin`. 13 links, 12 revolute joints.
- **Joints:** `<leg>_hip_yaw`, `<leg>_hip_pitch`, `<leg>_knee`.
  - Each axis is the actual STS3215 shaft from the CAD.
  - Positive rotation is right-handed about the output-horn direction.
- **Zero pose:** every joint at 0 is the CAD pose. All four legs are identical modules rotated 90°, not mirrored, so the sign convention is the same for every leg relative to its own corner.

### Joint constraints

Each joint has two layers of limits:

| Joint | Hard `<limit>` (mechanical) | **Recommended** (soft, `<safety_controller>`) |
|---|---|---|
| hip_yaw | −107° … +103° | **−90° … +90°** |
| hip_pitch | −123° … +61° | **−90° … +50°** |
| knee | −49° … +180° | **−20° … +150°** |

- **Hard limits:** each joint rotated alone in Inventor until real parts collide, with all the links below it moving along and the cable plugs included, minus a 3° margin (`../reference/joint_limits_detail.json`).
  - What sets them: yaw, the U-bracket back vs the yaw servo; pitch, the shin vs the bottom plate (−) and the thigh vs the yaw U-bracket (+); knee, the shin vs the yaw U-bracket.
- **Recommended limits:** tested collision-free with all three leg joints at their extremes together, all 27 combinations (`../reference/combined_check.log`).
  - Hip pitch and knee interact: with pitch below about −60°, knee −40° folds the shin into the hip. That's why the recommended knee minimum is −20° and the pitch minimum −90°.
  - The knee stays below +150°, clear of the STS3215's ±180° wrap (if servo center = zero pose).
- **Where they apply:**
  - `twist_2_0.urdf` / Gazebo: hard `<limit>` + soft `<safety_controller>`. The Gazebo xacro also clamps position commands to the recommended range (`command_interface` min/max).
  - `twist_2_0_isaac.urdf`: Isaac ignores soft limits, so `<limit>` there is the recommended range.
- **Command the robot inside the recommended range.** Also enable self-collision in the simulator for other leg/body contacts.
- **Effort and velocity:** 1.91 N·m and 4.71 rad/s (STS3215 at 7.4 V: 19.5 kg·cm stall, 0.222 s/60°). For the 12 V STS3215, set effort to 2.94 N·m.

## Mass

Total 1.130 kg, with inertias integrated from the CAD meshes. Every one of the 71 parts in the assembly is assigned to a link (the exporter checks this).

| Link | Mass |
|---|---|
| base_link (plates, 4 servos, board, cable plugs) | 403.8 g |
| each hip (2 U-brackets, 2 idler spacers) | 19.9 g |
| each thigh (2 servos, 2 H-plates, cable plugs) | 124.8 g |
| each shin (2 clamps, foot, idler spacer) | 36.9 g |

**Assumptions** (change them in `export_urdf.py` and re-run):

| Part | Value used |
|---|---|
| STS3215 | 55 g (datasheet) |
| Aluminium 6061 (brackets, plates) | 2.70 g/cm³ |
| Idler spacers (3D-printed PETG, 100 % infill) | 1.27 g/cm³ |
| Foot (3D-printed PLA, ~50 % infill) | 0.75 g/cm³ |
| ESP32-P4 dev kit | 40 g |
| Brass standoffs | 8.5 g/cm³ |
| Plugged cable ends | 4 g per servo |

Screws, the cable runs to the board and the battery are not included. Add the battery as a fixed link when you know where it goes.

## Checks done (`../scripts/validate_urdf.py`)

- Link tree is valid, every mesh file exists, and every inertia is positive definite and satisfies the triangle inequality.
- The URDF robot rebuilt at zero pose matches Inventor's own whole-assembly STL export to 0.00 mm.
- `../reference/urdf_preview.png` shows the zero pose and a test pose with every joint moved.
- Not run here, because this PC has no ROS or Isaac: `check_urdf`, RViz, Gazebo, Isaac Sim. Please run the steps below on the team machine.

## Use

### ROS 2 (Jazzy + Gazebo Harmonic)

```bash
# copy this folder into <ws>/src/, then
colcon build --packages-select twist_2_0_description && source install/setup.bash
check_urdf src/twist_2_0_description/urdf/twist_2_0.urdf
ros2 launch twist_2_0_description display.launch.py    # RViz + sliders
ros2 launch twist_2_0_description gazebo.launch.py     # gz-sim + ros2_control
```

### Isaac Sim

1. Use the URDF importer (*File → Import*) on `urdf/twist_2_0_isaac.urdf`.
2. Untick *Fix Base Link*.
3. Set *Joint Drive Type* to *Position*.
4. Collision is set to convex hulls already, so *Convex Decomposition* isn't needed.
