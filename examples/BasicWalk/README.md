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

Open Serial Monitor at 115200 baud for MPU6050 startup diagnostics. Successful
initialization prints `MPU6050 initialized successfully. DMP ready; FIFO packet size: 28`.
This confirms initialization; the first motion sample arrives afterward.
If startup fails, the log identifies the connection check, DMP error code, or
invalid FIFO packet size. DMP error code 1 in the bundled library means the
firmware memory upload/verification failed. If no `Initializing MPU6050` message
appears, check the standing feedback: this example waits for all ten servos to
reach the standing pose before calling `beginIMU()`.

The controller preserves the legs' half-cycle phase separation, uses hip-local
foot coordinates, and maps right hip pitch to the model's opposite axis before
applying calibration. Both hip-roll axes point along model -X. The thigh and
knee-to-ankle links are each 78.65 mm, matching the v2 robot model.
IMU correction uses fresh, finite samples, is capped at
0.10 radians, and is applied once. Invalid IK or out-of-range targets stop gait
updates and retain the last commanded positions instead of sending all servos
to their centers.

Walking uses a two-second cycle, 20 mm stride and 10 mm foot lift, with a
125 mm nominal hip-to-ankle vertical distance. Before each swing, a 300 ms
double-support interval shifts the body toward the support leg (18 mm to
either side). Lateral foot targets have the opposite sign to body translation.
Both planted feet have the same backward velocity during double support.
The 2000 ticks/s servo speed limit exceeds the nominal gait's measured
target peak of about 940 ticks/s. These values are in `src/gait.h` and
`src/gait.cpp`. It still uses the repository's simplified IK
and experimental balance controller, not the upstream trained walking policy.
The standing pose has been tested on your robot; the walking gait and IMU
mounting conventions still need physical validation with the robot supported.

## Knee and ankle feedback

`LOG_LEG_FEEDBACK` in the sketch currently enables diagnostic serial output.
One servo is read every 100 ms, cycling through IDs 23 (left knee), 24 (left
ankle), 13 (right knee), and 14 (right ankle). For example:

```text
LEG phase=0.32 id=23 target=3009 actual=2875 error=134
```

This is an illustrative tracking error, not a measurement from your robot.
`target` is the last position sent to that servo; `actual` is `ReadPos()`;
`error` is target minus actual. Units are ticks (about 11.38 ticks/degree).
`READ_FAILED` indicates that no valid position was returned. Reads are
sequential snapshots, not simultaneous measurements or a closed-loop correction.
Set `LOG_LEG_FEEDBACK = false` after diagnosis to remove the extra bus/serial work.

After the initial transition, nominal swing moves the knee from about 72.7
to 84.5 degrees and the ankle from -31.8 to -42.3 degrees between lift-off
and mid-swing. The right leg follows half a cycle later. IMU pitch correction
can change ankle targets. Check several full cycles with the body supported
and feet clear of the floor. Persistent target/actual differences indicate a
tracking problem; matching feedback with no corresponding physical joint
motion calls for checking motor IDs and assembly/calibration. If these motions
work unloaded but fail on the floor, examine support, loading and contact.

The full-URDF check predicts about 10.7 mm of foot-frame rise from lift-off to
mid-swing with a fixed body. It also reveals a limitation of the simplified
hip-local gait: the lateral shift produces about 16 mm of left/right foot-frame
height difference at the nominal double-support endpoints with a level body.
Thus the scheduled double-support phase does not guarantee both soles are
flat on the floor. Servo feedback alone cannot establish clearance or balance.

Run the independent full-model knee/ankle check with NumPy, SciPy and a host
`g++` installed:

```sh
python -m unittest discover -s tests -p test_leg_model.py
```

Host-side regression check (from the repository root):

```sh
g++ -std=c++11 -Wall -Wextra -Werror -Itests/host -Isrc tests/gait_test.cpp src/gait.cpp src/kinematics.cpp src/calibration.cpp -o /tmp/open-duck-gait-test
/tmp/open-duck-gait-test
```

This checks kinematics, target continuity, support timing and joint limits;
it does not simulate contact forces or establish physical walking stability.
