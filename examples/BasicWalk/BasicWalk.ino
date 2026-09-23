/*
  BasicWalk - AMB82-Mini / Open Duck Mini v2.

  Move to the BasicStand pose, wait for arrival, and hold for three seconds.
  Initialize/calibrate the MPU6050 during that hold, supported, still and level.
  Once both finish, blend into the experimental IK gait over two seconds.
  The gait in src/gait.h uses 20 mm steps and 20 mm foot lift, with a
  brief peak-height hold during each swing (full lift after the blend).

  Requires SCServo, AmebaPro_OpenDuck (updated src files), and the MPU6050
  library bundled with the AmebaPro2 Wire library. Serial2: 1 Mbps through the
  servo bus adapter. Servos must be in position mode with torque enabled.
*/
#include <amb82mini.h>

constexpr uint32_t STAND_HOLD_MS = 3000;
constexpr uint32_t WALK_TRANSITION_MS = 2000;
constexpr uint32_t STAND_TIMEOUT_MS = 30000;
constexpr uint32_t FEEDBACK_INTERVAL_MS = 100;
// Temporary knee/ankle diagnostics. One read per interval limits bus traffic.
constexpr bool LOG_LEG_FEEDBACK = true;
constexpr uint32_t LEG_FEEDBACK_INTERVAL_MS = 100;

AMB82Mini robot;
enum class StartupState { MovingToStand, HoldingStand, Walking, Stopped };
StartupState state = StartupState::Stopped;
uint32_t standCommandMs = 0;
uint32_t holdStartMs = 0;
uint32_t lastFeedbackMs = 0;
uint32_t lastLegFeedbackMs = 0;

void logLegFeedback(uint32_t now) {
    if (!LOG_LEG_FEEDBACK || now - lastLegFeedbackMs < LEG_FEEDBACK_INTERVAL_MS) return;
    lastLegFeedbackMs = now;
    // Calibration indices: left knee/ankle, right knee/ankle.
    static const uint8_t joints[] = {3, 4, 8, 9};
    static uint8_t nextJoint = 0;
    const uint8_t index = joints[nextJoint];
    nextJoint = (nextJoint + 1) % 4;
    int target, actual;
    const bool ok = robot.readJointFeedback(index, target, actual);
    Serial.print("LEG phase="); Serial.print(robot.getGaitPhase());
    Serial.print(" id="); Serial.print(robot.calibration()[index].id);
    Serial.print(" target="); Serial.print(target);
    Serial.print(" actual=");
    if (ok) {
        Serial.print(actual);
        Serial.print(" error="); Serial.println(target - actual);
    } else {
        Serial.println("READ_FAILED");
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000, SERIAL_8N1);
    robot.beginServos(Serial2);
    if (!robot.stand()) {
        Serial.println("Stopped: check leg servo communication and calibration.");
        return;
    }
    standCommandMs = millis();
    lastFeedbackMs = standCommandMs;
    state = StartupState::MovingToStand;
    Serial.println("Moving to BasicStand pose; waiting for all ten servos.");
}

void loop() {
    if (state == StartupState::Stopped) {
        delay(10);
        return;
    }
    robot.loop();  // No gait motion until startWalking() succeeds.
    const uint32_t now = millis();

    if (state == StartupState::Walking) {
        if (!robot.isWalking()) {
            state = StartupState::Stopped;
            Serial.println("Stopped: invalid walking target; holding last servo targets.");
        }
        logLegFeedback(now);
        return;
    }
    if (now - standCommandMs >= STAND_TIMEOUT_MS) {
        state = StartupState::Stopped;
        Serial.println("Stopped: standing timeout; walking was not started.");
        return;
    }
    if (now - lastFeedbackMs < FEEDBACK_INTERVAL_MS) return;
    lastFeedbackMs = now;
    if (!robot.standingReached()) {
        // Require a continuous hold within 20 ticks (~1.8 degrees) per servo.
        state = StartupState::MovingToStand;
        return;
    }
    if (state == StartupState::MovingToStand) {
        holdStartMs = now;
        state = StartupState::HoldingStand;
        Serial.println("Standing reached. Holding still for 3 seconds.");
        // Calibration time counts toward the hold; servos retain their targets.
        Serial.println("Keep robot supported, still and level: initializing IMU.");
        if (!robot.beginIMU()) {
            state = StartupState::Stopped;
            Serial.println("Stopped: MPU6050 initialization failed; holding standing pose.");
            return;
        }
        // beginIMU() blocks. Recheck timeout and servo feedback next loop
        // using fresh time before allowing walking. A longer calibration
        // extends the standing phase beyond three seconds.
        return;
    } else if (now - holdStartMs >= STAND_HOLD_MS) {
        if (robot.startWalking(WALK_TRANSITION_MS)) {
            state = StartupState::Walking;
            Serial.println("Starting walking with a 2-second transition.");
            if (LOG_LEG_FEEDBACK) {
                Serial.println("LEG feedback in ticks: 23=L knee, 24=L ankle, 13=R knee, 14=R ankle; 11.38 ticks/degree.");
            }
        } else {
            state = StartupState::Stopped;
            Serial.println("Stopped: unable to start walking from standing pose.");
        }
    }
}
