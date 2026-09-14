# AmebaPro_OpenDuck

Note: BNO055 currently not supported by AmebaPro2, replaced by MPU6050

## OpenDuck Mini v2
### source: [Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini)

### PCBs
| PCB                      |    Picture    |
|--------------------------|---------------|
| Front open-case view     | <img width="25%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/OpenDuck_mini_v2_front_open.png?raw=true"> |
| I2C-GPIO extension board | <img width="25%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/OpenDuck_mini_v2_I2C_GPIO.png?raw=true"> |
| Main board (RPi4B)       | <img width="25%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/OpenDuck_mini_v2_RPi4B.png?raw=true"> |)
| Serial Bus Servo Adapter | <img width="25%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/OpenDuck_mini_v2_serial_bus_servo_adapter.png?raw=true"> |
| BNO055 9-axis IMU        | <img width="25%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/OpenDuck_mini_v2_BNO055_IMU.png?raw=true"> |

---
### Servo: STS3215

#### Library : [Arduino STS_Servos driver](https://github.com/DeltaEngine/FeetechStsServos)

#### Sketch : [STS_Servos_ListServos.ino](https://github.com/rkuo2000/EdgeAI-AMB82mini/blob/main/Arduino/HUB8735-ultra/STS_Servos_ListServos/STS_Servos_ListServos.ino)

#### Motor IDs
```
{
    "left_hip_yaw": 20,
    "left_hip_roll": 21,
    "left_hip_pitch": 22,
    "left_knee": 23,
    "left_ankle": 24,
    "neck_pitch": 30,
    "head_pitch": 31,
    "head_yaw": 32,
    "head_roll": 33,
    "right_hip_yaw": 10,
    "right_hip_roll": 11,
    "right_hip_pitch": 12,
    "right_knee": 13,
    "right_ankle": 14,
}
```

---
## Ameba82-Mini
### [Hub8735-Ultra](https://github.com/ideashatch/HUB-8735)
<img width="10%" src="https://github.com/rkuo2000/AmebaPro_OpenDuck_Mini/blob/main/pics/Hub8735_Ultra.png?raw=true">

### Replace RPi4B with AMB82-Mini
Prompt for ChatGPT Pro: <br>
```
given https://github.com/apirrone/Open_Duck_Mini, if I use AMB82-Mini to replace the RPi-4B, how can I build servo control by bipedal kinematics ?
```

Yes. For **Open Duck Mini v2**, replacing the Raspberry Pi with an **AMB82-Mini** is practical if you change the software architecture: instead of trying to port the existing Python/Linux runtime directly, implement a small real-time **gait generator → inverse kinematics → joint calibration → STS3215 SyncWrite** controller in C/C++ on the AMB82.

The current robot has **5 actuated joints per leg**:

```text
Left leg                        Right leg
------------------------------------------------
left_hip_yaw                    right_hip_yaw
left_hip_roll                   right_hip_roll
left_hip_pitch                  right_hip_pitch
left_knee                       right_knee
left_ankle                      right_ankle
```

The official MuJoCo model confirms that ordering. ([GitHub][1])

### 1. Recommended AMB82 architecture

I would structure the AMB82 firmware like this:

```text
                    AMB82-MINI
                         │
             ┌───────────┴────────────┐
             │                        │
       Gait Generator              IMU
        100-200 Hz             roll/pitch/yaw
             │                        │
             └──────────┬─────────────┘
                        ↓
                Desired feet pose
            Left XYZ / Right XYZ
                        ↓
              Bipedal Kinematics
               Inverse Kinematics
                        ↓
        ┌──────────────────────────────┐
        │ L hip yaw     R hip yaw      │
        │ L hip roll    R hip roll     │
        │ L hip pitch   R hip pitch    │
        │ L knee        R knee         │
        │ L ankle       R ankle        │
        └──────────────┬───────────────┘
                       ↓
              Offset/sign/limit
                       ↓
            radians → STS3215 units
                       ↓
             UART 1 Mbps SyncWrite
                       ↓
          ┌────────────┴─────────────┐
          │                          │
       Left leg                  Right leg
       5 motors                   5 motors
```

AMB82-Mini provides UART, I²C, SPI, PWM, Wi-Fi/BLE, etc., so the peripheral side is suitable for this design. ([GitHub][2])

The important difference is that the Open Duck's STS3215 motors are **serial smart servos**, not conventional PWM servos. STS3215 normally uses a 1 Mbps half-duplex serial bus and a 4096-count encoder. ([GitHub][3])

---
### 2. Don't control the servos directly from gait phase

Create this separation:

```text
velocity command
       ↓
   gait phase
       ↓
 foot trajectory
       ↓
 IK solver
       ↓
 joint angles (rad)
       ↓
 calibration
       ↓
 servo position
```

For example, your gait generator shouldn't say:

```cpp
servo1 = 2200;
servo2 = 1700;
servo3 = 2500;
```

Instead it should say:

```cpp
leftFoot  = {xL, yL, zL};
rightFoot = {xR, yR, zR};

leftJoint  = legIK(leftFoot);
rightJoint = legIK(rightFoot);
```

That makes the walking geometry independent from servo calibration.

---
### 3. Open Duck Mini leg geometry

The MuJoCo model gives approximately:

```text
Hip
 │
 │ 78.65 mm
 │
 Knee
 │
 │ 78.65 mm
 │
 Ankle
```

The XML has successive knee and ankle offsets of about **0.07865 m**. ([GitHub][4])

Therefore, for a first AMB82 implementation, use:

```cpp
const float L1 = 0.07865f;  // thigh
const float L2 = 0.07865f;  // lower leg
```

I would later measure the actual assembled robot and adjust these values because the MuJoCo `left_foot`/`right_foot` sites include additional foot offsets.

---
### 4. Simplify the 5-DOF leg

Each Open Duck Mini leg has:

```text
             hip yaw
                │
             hip roll
                │
             hip pitch
                │
               knee
                │
              ankle
```

Notice something important:

**there is no ankle-roll joint.**

So this isn't a conventional humanoid 6-DOF leg. You cannot independently specify all six components of foot pose.

A useful first controller therefore controls:

```text
x       forward/back
y       left/right
z       height
yaw     foot/body heading
pitch   foot pitch
```

Five variables → five joints.

For initial walking, simplify even more:

```text
foot_yaw   = 0
foot_pitch = 0
```

and calculate XYZ.

---
### 5. Core inverse kinematics

Define the hip coordinate system:

```text
          +Z
           ↑
           │
           ● hip
          /│
         / │
        /  │
       ● knee
       │
       │
       ● foot

+X = forward
+Y = robot left
+Z = upward
```

A standing foot might therefore be approximately:

```cpp
FootTarget left = {
    0.0,       // x
    +0.035,    // y
    -0.145     // z
};
```

#### Hip roll

First reduce the XYZ problem into the sagittal plane:

$$
qroll​=atan2 (y, -z)
$$

Then effective vertical distance is:

$$
z_s=\sqrt{y^2+z^2}
$$

---
### 6. Hip pitch + knee IK

Now solve the usual 2-link leg:

$$
d=\sqrt{x^2+z_s^2}
$$

Clamp it:

$$
|L_1-L_2|+\epsilon < d < L_1+L_2-\epsilon
$$

Then:

$$
\cos(q_k)=
\frac{L_1^2+L_2^2-d^2}
{2L_1L_2}
$$

One useful knee convention is:

$$
q_{knee}=\pi-\cos^{-1}(\cos(q_k))
$$

For the hip:

$$
\alpha=atan2 (x,z_s)
$$

$$
\beta=
\cos^{-1}
\left(
\frac{L_1^2+d^2-L_2^2}
{2L_1d}
\right)
$$

and approximately:

$$
q_{hipPitch}=\alpha-\beta
$$

Then if you want the foot approximately parallel to the ground:

$$
q_{ankle}=-(q_{hipPitch}+q_{knee})
$$

The exact signs will depend on the Open Duck left/right servo installation.

---
### 7. AMB82 C++ IK

A small implementation could look like this:

```cpp
#include <Arduino.h>
#include <math.h>

constexpr float L1 = 0.07865f;
constexpr float L2 = 0.07865f;

struct FootTarget {
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
};

struct LegAngles {
    float hipYaw;
    float hipRoll;
    float hipPitch;
    float knee;
    float ankle;
};

float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

bool legIK(const FootTarget &foot, LegAngles &q)
{
    float x = foot.x;
    float y = foot.y;
    float z = foot.z;

    // -----------------------------------------
    // 1. hip yaw
    // -----------------------------------------
    q.hipYaw = foot.yaw;

    // Transform target into frame after yaw.
    float cy = cosf(-q.hipYaw);
    float sy = sinf(-q.hipYaw);

    float xr = cy * x - sy * y;
    float yr = sy * x + cy * y;

    // -----------------------------------------
    // 2. hip roll
    // -----------------------------------------
    q.hipRoll = atan2f(yr, -z);

    // sagittal-plane vertical distance
    float zs = sqrtf(yr * yr + z * z);

    // -----------------------------------------
    // 3. planar 2-link IK
    // -----------------------------------------
    float d = sqrtf(xr * xr + zs * zs);

    const float minD = fabsf(L1 - L2) + 0.001f;
    const float maxD = L1 + L2 - 0.001f;

    if (d < minD || d > maxD)
        return false;

    float ck =
        (L1 * L1 + L2 * L2 - d * d) /
        (2.0f * L1 * L2);

    ck = clampf(ck, -1.0f, 1.0f);

    float kneeInternal = acosf(ck);

    q.knee = PI - kneeInternal;

    float ch =
        (L1 * L1 + d * d - L2 * L2) /
        (2.0f * L1 * d);

    ch = clampf(ch, -1.0f, 1.0f);

    float beta = acosf(ch);
    float alpha = atan2f(xr, zs);

    q.hipPitch = alpha - beta;

    // Keep desired foot pitch
    q.ankle =
        foot.pitch -
        q.hipPitch -
        q.knee;

    return true;
}
```

This is the correct kind of solver for an AMB82 implementation, but the final left/right `sign` mappings need to be obtained from your actual servo calibration.

---
### 8. Very important: joint sign/offset mapping

Don't send these IK values directly to the STS3215.

Use a calibration table.

For example:

```cpp
enum Joint {
    L_HIP_YAW,
    L_HIP_ROLL,
    L_HIP_PITCH,
    L_KNEE,
    L_ANKLE,

    R_HIP_YAW,
    R_HIP_ROLL,
    R_HIP_PITCH,
    R_KNEE,
    R_ANKLE,

    NUM_JOINTS
};

struct JointCalibration {
    uint8_t id;

    float direction;

    int center;

    int offset;

    float minAngle;
    float maxAngle;
};
```

Something conceptually like:

```cpp
JointCalibration joint[NUM_JOINTS] = {

 // ID sign center offset min       max

 { 1, +1, 2048, 0, -0.524, +0.524 }, // L hip yaw
 { 2, +1, 2048, 0, -0.436, +0.436 }, // L hip roll
 { 3, -1, 2048, 0, -1.222, +0.524 }, // L hip pitch
 { 4, +1, 2048, 0, -1.571, +1.571 }, // L knee
 { 5, -1, 2048, 0, -1.571, +1.571 },

 { 6, -1, 2048, 0, -0.524, +0.524 },
 { 7, -1, 2048, 0, -0.436, +0.436 },
 { 8, +1, 2048, 0, -0.524, +1.222 },
 { 9, -1, 2048, 0, -1.571, +1.571 },
 {10, +1, 2048, 0, -1.571, +1.571 }
};
```

**Those ID/direction entries are examples, not the official Open Duck wiring.** You'll determine them during calibration.

The limits, however, correspond closely to the MuJoCo model. The model specifies hip yaw around ±30°, hip roll around ±25°, asymmetric hip-pitch ranges, and knee/ankle around ±90°. ([GitHub][5])

---
### 9. Convert radians to STS3215 positions

STS3215 has:

```text
4096 counts / 360°
2048 ≈ nominal center
```

so:

$$
ticks =
center + offset +
direction \times q
\frac{4096}{2\pi}
$$

Implement it as:

```cpp
int angleToServo(
    float angle,
    const JointCalibration &j)
{
    angle = clampf(
        angle,
        j.minAngle,
        j.maxAngle
    );

    constexpr float RAD_TO_TICKS =
        4096.0f / (2.0f * PI);

    int ticks =
        j.center +
        j.offset +
        (int)(
            j.direction *
            angle *
            RAD_TO_TICKS
        );

    if (ticks < 0)
        ticks = 0;

    if (ticks > 4095)
        ticks = 4095;

    return ticks;
}
```

The 4096-step resolution and standard 1 Mbps rate are documented for the STS3215. ([GitHub][3])

---
### 10. Use SyncWrite, not ten individual writes

This is particularly important for walking.

Bad:

```text
servo1 command
5 ms
servo2 command
5 ms
servo3 command
...
```

By the time servo 10 gets its target, the first joint has already started moving.

Instead:

```text
Calculate all 10 angles
        ↓
build ONE SyncWrite packet
        ↓
broadcast
        ↓
all 10 goal positions update together
```

STS protocol provides `SYNC WRITE` specifically for synchronized multi-servo commands. ([GitHub][6])

The STS3215 registers include:

```text
40  torque enable
41  acceleration
42  goal position low
43  goal position high
44  goal time low
45  goal time high
46  goal speed low
47  goal speed high

56  present position
57
58  present speed
...
69  present current
70
```

([GitHub][7])

A synchronized packet can therefore update something like:

```text
ACC + Position + Time + Speed
```

for all ten motors at once.

---
### 11. AMB82 control loop

I'd start at **100 Hz**.

```cpp
void loop()
{
    static uint32_t lastControl = 0;

    uint32_t now = micros();

    if (now - lastControl >= 10000) {

        lastControl += 10000;

        // 1. Update gait phase
        updateGait(0.010f);

        // 2. Generate desired feet positions
        FootTarget lf;
        FootTarget rf;

        generateFeet(lf, rf);

        // 3. IK
        LegAngles qL;
        LegAngles qR;

        bool okL = legIK(lf, qL);
        bool okR = legIK(rf, qR);

        if (!okL || !okR) {
            emergencyHold();
            return;
        }

        // 4. map to 10 joints
        float q[10];

        q[L_HIP_YAW]   = qL.hipYaw;
        q[L_HIP_ROLL]  = qL.hipRoll;
        q[L_HIP_PITCH] = qL.hipPitch;
        q[L_KNEE]      = qL.knee;
        q[L_ANKLE]     = qL.ankle;

        q[R_HIP_YAW]   = qR.hipYaw;
        q[R_HIP_ROLL]  = qR.hipRoll;
        q[R_HIP_PITCH] = qR.hipPitch;
        q[R_KNEE]      = qR.knee;
        q[R_ANKLE]     = qR.ankle;

        // 5. radians -> servo ticks
        int position[10];

        for (int i = 0; i < 10; i++) {
            position[i] =
                angleToServo(q[i], joint[i]);
        }

        // 6. ONE serial packet
        syncWriteServos(position);
    }
}
```

This gives:

```text
100 Hz

10 ms
 ├── IMU
 ├── gait
 ├── IK left
 ├── IK right
 ├── balance correction
 ├── joint limits
 └── STS SyncWrite
```

That workload is very reasonable for the AMB82.

---
### 12. Gait generation

For initial testing, don't attempt Open Duck's RL walking policy yet.

Start with a deterministic walking trajectory.

Let:

$$
\phi=0\ldots1
$$

represent one walking cycle.

Left/right legs are separated by 180°:

```cpp
float phaseL = phase;
float phaseR = fmodf(phase + 0.5f, 1.0f);
```

A very basic swing trajectory can use:

$$
x=A\sin(2\pi\phi)
$$

and foot lift:

$$
z =
z_0 +
\begin{cases}
H\sin(2\pi\phi),&0<\phi<0.5\\
0,&0.5\leq\phi<1
\end{cases}
$$

I prefer defining explicit **stance** and **swing** phases, though.

For example:

```cpp
void footTrajectory(
    float phase,
    float stepLength,
    float stepHeight,
    float z0,
    FootTarget &foot)
{
    if (phase < 0.5f) {

        // STANCE
        float s = phase / 0.5f;

        foot.x =
            stepLength * (0.5f - s);

        foot.z = z0;

    } else {

        // SWING
        float s =
            (phase - 0.5f) / 0.5f;

        foot.x =
            stepLength * (-0.5f + s);

        foot.z =
            z0 +
            stepHeight *
            sinf(PI * s);
    }

    foot.yaw = 0;
    foot.pitch = 0;
}
```

Then:

```cpp
void generateFeet(
    FootTarget &left,
    FootTarget &right)
{
    float pL = gaitPhase;
    float pR = fmodf(
        gaitPhase + 0.5f,
        1.0f
    );

    footTrajectory(
        pL,
        0.030f,     // 30 mm stride
        0.012f,     // 12 mm lift
        -0.140f,
        left
    );

    footTrajectory(
        pR,
        0.030f,
        0.012f,
        -0.140f,
        right
    );

    left.y  = +0.035f;
    right.y = -0.035f;
}
```

For initial bench testing I'd use an even smaller step amplitude than this.

---
### 13. But this alone will probably fall over

Pure IK gives you:

```text
"put the feet here"
```

It does **not** solve:

```text
"keep the center of mass above the support foot."
```

That is the next important component.

A biped walking controller should become:

```text
            velocity command
                   │
                   ↓
            gait generator
                   │
          ┌────────┴────────┐
          ↓                 ↓
       L foot            R foot
          │                 │
          └────────┬────────┘
                   ↓
                  IK
                   │
                   ↓
             nominal joints
                   │
                   +
                   │
           ┌───────┴───────┐
           │               │
         IMU roll        IMU pitch
           │               │
           ↓               ↓
          PID             PID
           │               │
           └───────┬───────┘
                   ↓
          balance correction
                   ↓
              servo command
```

---
### 14. Start with lateral body shifting

Before lifting a foot, shift the trunk toward the support leg.

For example:

```text
Phase 0
Both feet down
body centered

Phase 1
shift body LEFT

Phase 2
lift RIGHT foot

Phase 3
move RIGHT foot forward

Phase 4
put RIGHT foot down

Phase 5
shift body RIGHT

Phase 6
lift LEFT foot

Phase 7
move LEFT foot forward
```

You can accomplish the initial body shift by modifying both foot Y targets.

Conceptually:

```cpp
float bodyShift =
    0.010f * sinf(2.0f * PI * phase);

left.y =
    +HIP_WIDTH / 2 - bodyShift;

right.y =
    -HIP_WIDTH / 2 - bodyShift;
```

This is much easier to debug than jumping directly to dynamically balanced locomotion.

---
### 15. Add IMU stabilization

AMB82 reads your IMU:

```text
desired roll = 0
desired pitch = desired torso pitch
```

Calculate:

```cpp
float rollError =
    desiredRoll - imuRoll;

float pitchError =
    desiredPitch - imuPitch;
```

and PID:

```cpp
float rollCorrection =
    kpRoll * rollError +
    kdRoll * rollRate;

float pitchCorrection =
    kpPitch * pitchError +
    kdPitch * pitchRate;
```

Apply small corrections:

```cpp
qL.hipRoll  += rollCorrection;
qR.hipRoll  += rollCorrection;

qL.hipPitch += pitchCorrection;
qR.hipPitch += pitchCorrection;

qL.ankle    -= pitchCorrection;
qR.ankle    -= pitchCorrection;
```

Keep these corrections small and clamp everything to the joint limits.

---
### 16. How this differs from the existing Open Duck software

This matters because the official project currently uses **RL/sim2real** rather than only classical IK. The main repository states that its onboard runtime runs policies, while the runtime repo depends on Python packages including ONNX Runtime, NumPy, SciPy, `rustypot`, and `pypot`. ([GitHub][8])

Therefore I would **not** try to duplicate this on AMB82 first:

```text
MuJoCo-trained RL policy
        ↓
ONNX Runtime
        ↓
joint policy output
```

Instead, do this:

```text
       Phase 1
AMB82 + STS3215
basic serial servo control

       ↓

       Phase 2
AMB82 + calibration
joint angle control

       ↓

       Phase 3
forward/inverse kinematics
standing pose

       ↓

       Phase 4
weight shifting
single foot lifting

       ↓

       Phase 5
trajectory gait
slow walking

       ↓

       Phase 6
IMU feedback
balanced walking

       ↓

       Phase 7
optional neural policy
```

This isolates hardware bugs from locomotion bugs.

---
### 17. Hardware connection I recommend

Conceptually:

```text
          AMB82-MINI
        ┌─────────────┐
        │             │
        │ UART TX ────┼────┐
        │ UART RX ────┼────┤
        │             │    ↓
        │ I2C ────────┼── IMU
        │             │
        └─────────────┘
                         │
                half-duplex interface
                         │
                    STS BUS
                         │
        ┌────────────────┼─────────────────┐
        │       │        │       │         │
      Servo1  Servo2   Servo3  ...      Servo10
```

The motors need their own correctly sized supply; don't power the servo chain from the AMB82 board. Tie the logic/interface ground and servo-controller ground appropriately according to your interface-board design.

---
### 18. I would keep the AMB82 responsible for these things

This is where AMB82 could actually be a nice replacement:

```text
AMB82-Mini
│
├── 100-200 Hz real-time locomotion loop
│   ├── gait generator
│   ├── biped IK
│   ├── IMU feedback
│   └── STS3215 UART
│
├── camera
│
├── onboard AI/NPU
│   ├── person detection
│   ├── object detection
│   └── gesture detection
│
├── Wi-Fi
│
└── audio
```

That's a good match for the AMB82's native camera/video/NN capabilities and peripheral interfaces. ([GitHub][2])

I would initially leave **RL training, MuJoCo, and large neural locomotion policy development** on a PC.

---
### 19. The firmware architecture I'd build

```text
OpenDuck_AMB82/
│
├── OpenDuck_AMB82.ino
│
├── robot_config.h
│   ├── servo IDs
│   ├── servo offsets
│   ├── directions
│   └── joint limits
│
├── sts3215_bus.cpp
├── sts3215_bus.h
│   ├── packet
│   ├── SyncWrite
│   ├── SyncRead
│   └── error handling
│
├── kinematics.cpp
├── kinematics.h
│   ├── legIK()
│   └── legFK()
│
├── gait.cpp
├── gait.h
│   ├── phase
│   ├── stance trajectory
│   ├── swing trajectory
│   └── turning
│
├── imu.cpp
├── imu.h
│
├── balance.cpp
├── balance.h
│   ├── roll PID
│   └── pitch PID
│
└── safety.cpp
    ├── joint limits
    ├── servo temperature
    ├── servo current
    ├── fall detection
    └── torque disable
```

This is a much cleaner embedded replacement for the Pi software.

## The key design choice

For your AMB82 version of Open Duck Mini, I recommend **not** making the AMB82 imitate the Raspberry Pi.

Make it a dedicated robot controller:

```text
          Original = RPi-4B
MuJoCo/RL → Python/Linux → servo library → STS3215

              ↓

          AMB82-Mini version

Command
  ↓
Gait trajectory
  ↓
Foot XYZ
  ↓
5-DOF leg IK
  ↓
10 joint angles
  ↓
IMU corrections
  ↓
servo calibration
  ↓
STS3215 SyncWrite @ 1 Mbps
  ↓
Open Duck Mini
```

That gives you a very lightweight, deterministic controller without requiring Linux. It also leaves the AMB82's camera/NPU free for the vision functions you're interested in adding.

The next useful step would be to create an **actual compilable AMB82-Mini Arduino project** containing `STS3215Bus`, the complete left/right 5-DOF IK, servo calibration, gait generator, and a simple serial command interface such as `stand`, `walk`, `stop`, `vx`, `vy`, and `yaw`. I can build that around the Open Duck Mini v2 joint geometry and make the servo-ID/offset table configurable.

[1]: https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/constants.py?utm_source=chatgpt.com "Open_Duck_Playground/playground/open_duck_mini_v2/constants.py at main · apirrone/Open_Duck_Playground · GitHub"
[2]: https://github.com/Ameba-AIoT/ameba-arduino-doc/blob/main/source/ameba_pro2/amb82-mini/Getting_Started/Getting%20Started%20with%20Ameba.rst?utm_source=chatgpt.com "ameba-arduino-doc/source/ameba_pro2/amb82-mini/Getting_Started/Getting Started with Ameba.rst at main · Ameba-AIoT/ameba-arduino-doc · GitHub"
[3]: https://github.com/beam-bots/feetech/blob/main/lib/feetech/control_table/sts3215.ex?utm_source=chatgpt.com "feetech/lib/feetech/control_table/sts3215.ex at main · beam-bots/feetech · GitHub"
[4]: https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml "Open_Duck_Playground/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml at main · apirrone/Open_Duck_Playground · GitHub"
[5]: https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml?utm_source=chatgpt.com "Open_Duck_Playground/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml at main · apirrone/Open_Duck_Playground · GitHub"
[6]: https://github.com/Mowibox/stm32-sts3215-lib/blob/main/docs/protocol_manual.md?utm_source=chatgpt.com "stm32-sts3215-lib/docs/protocol_manual.md at main · Mowibox/stm32-sts3215-lib · GitHub"
[7]: https://github.com/ftabcc/feetech_motor/blob/main/feetech_motor/sts3215_python/src/sts.py?utm_source=chatgpt.com "feetech_motor/feetech_motor/sts3215_python/src/sts.py at main · ftabcc/feetech_motor · GitHub"
[8]: https://github.com/apirrone/Open_Duck_Mini "GitHub - apirrone/Open_Duck_Mini: Making a mini version of the BDX droid. https://discord.gg/UtJZsgfQGe · GitHub"

