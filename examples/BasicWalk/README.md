# BasicWalk

Use the same AMB82-Mini / HUB8735, servo bus, IDs and calibrated centers as
`BasicStand`, plus the MPU6050 on I2C. Update **all library `src` files** in
your installed AmebaPro_OpenDuck library before building. This includes the
new `standing_pose.h`; copying only the example will not update the controller.

Keep the robot supported during startup and still and level during IMU
calibration. The example:

1. Sends the verified BasicStand targets at 200 ticks/s, acceleration 10.
2. Checks position feedback every 100 ms until all ten joints are within 20
   ticks (about 1.8 degrees) of their targets.
3. Holds that pose continuously for `STAND_HOLD_MS` (3000 ms). Leaving the
   tolerance restarts the hold. No gait phase advances during standing.
4. Initializes and calibrates the MPU6050 at the start of that hold. Calibration
   time counts toward the three seconds; if calibration takes longer, the robot
   keeps standing until it finishes. Calibration blocks position polling; servo
   feedback and the startup timeout are checked again afterward.
5. After both the hold and IMU initialization finish, calls `startWalking()`
   and blends the joint targets from the standing pose
   into the IK gait over `WALK_TRANSITION_MS` (2000 ms).

The initial angles are shared with BasicStand in `src/standing_pose.h`.
Startup reads only check communication and arrival; they never overwrite
calibration. Failure to reach and hold the standing pose within 30 seconds prevents walking.
IMU initialization failure prevents walking and leaves the standing targets in place. A failure leaves
existing servo torque/targets in place; it does not power off the servos.

The controller preserves the legs' half-cycle phase separation, uses hip-local
foot coordinates, and maps right hip pitch to the model's opposite axis before
applying calibration. IMU correction uses fresh, finite samples, is capped at
0.10 radians, and is applied once. Invalid IK or out-of-range targets stop gait
updates and retain the last commanded positions instead of sending all servos
to their centers.

Walking uses a one-second cycle, 30 mm stride and 12 mm foot lift, with a
2000 ticks/s servo speed limit (the nominal target motion peaks around 1300
ticks/s). It still uses the repository's simplified IK
and experimental balance controller, not the upstream trained walking policy.
The standing pose has been tested on your robot; the walking gait and IMU
mounting conventions still need physical validation with the robot supported.
