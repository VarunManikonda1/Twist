Twist 2.0 quadruped - servo angles recorded in Gazebo (Feetech STS3215, 12 servos)

Files
  forward_motor_angles_deg.csv    forward walk, servo angle in degrees (0-360, 180 = middle)
  backward_motor_angles_deg.csv   backward walk, servo angle in degrees
  forward_motor_ticks.csv         forward walk, STS3215 position steps (0-4095, 2048 = middle)
  backward_motor_ticks.csv        backward walk, STS3215 position steps
  motor_ids.txt                   which ID is which joint

Format
  One row every ~20 ms (50 Hz), 250 rows = 5 s (about 6 trot cycles, period 0.8 s).
  time_s = seconds from the start of the movement; ID1..ID12 = target position of each servo.
  Speed in sim: 0.08 m/s forward / backward.

Servo IDs
  ID1-3   front left  (hip yaw, hip pitch, knee)
  ID4-6   front right (hip yaw, hip pitch, knee)
  ID7-9   rear left   (hip yaw, hip pitch, knee)
  ID10-12 rear right  (hip yaw, hip pitch, knee)

Conversion
  steps = 2048 + direction * joint_angle_rad * 4096 / (2*pi) + offset
  degrees = steps * 360 / 4096
  Recorded with direction = +1 and offset = 0 for every servo, i.e. 2048 = joint at 0 in the URDF.

Before running on the robot
  Not calibrated yet. The real servos' zero position and turning direction must be checked
  (offset and direction per ID), otherwise the legs will go to wrong angles.
  Test with the robot held in the air first, and move slowly to the first row before playback.
