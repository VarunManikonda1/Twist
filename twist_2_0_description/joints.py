"""Joint naming order matching leg_controller (ros2_controllers.yaml)."""

JOINT_NAMES = [
    "FL_hip_yaw",
    "FL_hip_pitch",
    "FL_knee",
    "RR_hip_yaw",
    "RR_hip_pitch",
    "RR_knee",
    "RL_hip_yaw",
    "RL_hip_pitch",
    "RL_knee",
    "FR_hip_yaw",
    "FR_hip_pitch",
    "FR_knee",
]

LEG_INDICES = {
    "FL": (0, 1, 2),
    "RR": (3, 4, 5),
    "RL": (6, 7, 8),
    "FR": (9, 10, 11),
}
