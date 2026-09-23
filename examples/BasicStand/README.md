# BasicStand

Open `BasicStand.ino` in Arduino IDE and select your AMB82-Mini / HUB8735
board. Install SCServo and the AmebaPro_OpenDuck library used by `BasicWalk`.
The shared controller also needs the MPU6050 headers bundled with AmebaPro2
Wire, but this example does not initialize or read an IMU.
Make sure that library's `src` contains the matching files from this repository,
including your calibrated `calibration.cpp` and `standing_pose.h`; Arduino does not automatically
compile the repository's sibling `src` directory when opening an example.

Connect the servo bus adapter to Serial2 (GPIO 18 RX, GPIO 19 TX), with a shared
ground and an external servo supply. Configure the servos for 1 Mbps, position
mode and enabled torque. Leg IDs follow `docs/configure_motors.md`:

| Joint | Left ID | Right ID |
| --- | --- | --- |
| Hip yaw | 20 | 10 |
| Hip roll | 21 | 11 |
| Hip pitch | 22 | 12 |
| Knee | 23 | 13 |
| Ankle | 24 | 14 |

Verify centers, offsets, motor directions and angle limits for your assembly
before uploading. Support the robot during the initial motion. Open Serial
Monitor at 115200 baud; immediately at startup the sketch checks the pose
and reads all ten servos to verify communication, then sends one synchronized
position command at 200 ticks/s. These readings do not change calibration:
standing targets use the fixed values in `src/calibration.cpp`. A failed check
prevents the command; it does not disable existing servo torque. Reset the
board to retry after correcting a problem.

The pose uses the explicit v2 model joint angles from `init_pos` in
`experiments/v2/onnx_AWD_mujoco_motor_control.py`, rather than calculating a
shared pose for both legs with the simplified IK:

| Joint | Left angle (rad) | Right angle (rad) |
| --- | ---: | ---: |
| Hip yaw | 0.002 | -0.003 |
| Hip roll | 0.053 | -0.065 |
| Hip pitch | -0.630 | 0.635 |
| Knee | 1.368 | 1.379 |
| Ankle | -0.784 | -0.796 |

Both examples call `AMB82Mini::stand()` after `beginServos(Serial2)` to
validate calibration, check servo communication, and command the standing pose.

`STANDING_ANGLES` in `src/standing_pose.h` stores these model angles, shared
with BasicWalk; `STANDING_IDS` verifies their
mapping to calibration entries. Targets are converted with
`int(center + offset + direction * angle * 4096 / (2 * PI))`.
Centers, offsets and directions must map model joint coordinates to the actual
servos. The positive right hip-pitch angle is intentional; its calibration
limits now follow the model's -30° to +70° range. Update the installed library's
`calibration.cpp` too, or its old limit will reject this pose.

The [upstream hardware runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime/blob/v2/mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py)
uses these same angles and adds measured offsets without joint sign inversions.
The standard v2 defaults therefore use `direction = +1` for every joint.
Earlier versions of this example's calibration incorrectly reversed both hip
pitch joints, both ankles and right hip yaw. Update your installed library's
calibration signs as well as its limits; replacing the sketch alone is insufficient.

With centers 2048 and offsets zero, the expected targets are:

| Joint | Left ticks | Right ticks |
| --- | ---: | ---: |
| Hip yaw | 2049 | 2046 |
| Hip roll | 2082 | 2005 |
| Hip pitch | 1637 | 2461 |
| Knee | 2939 | 2946 |
| Ankle | 1536 | 1529 |

This is a bent-knee initialization pose, not a fully extended stance. A static
forward-kinematics check of the repository's `scene_position.xml` places the
foot frames about 161 mm below the base, with foot tilts of about 4–5 degrees.
Applying the old calibration sign inversions instead puts the foot frames near
base height with about 159–161 degrees of tilt. These are model calculations,
not a physical balance test or a measurement of your robot's servo zeros.

Out-of-limit angles and servo ticks are rejected. The servos hold their targets
internally indefinitely while powered with torque enabled; there is no
three-second hold limit or transition to walking. This example commands only the ten leg joints; the model's zero
head/antenna angles are not sent. It does not require an IMU or provide active
balance; actual unsupported stability depends on assembly and calibration.
