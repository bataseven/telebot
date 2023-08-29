#include "Robot.h"

Robot::Robot() {
    base = new MobileBase(
        FRONT_LEFT_STEP_PIN, FRONT_LEFT_DIR_PIN,
        FRONT_RIGHT_STEP_PIN, FRONT_RIGHT_DIR_PIN,
        REAR_LEFT_STEP_PIN, REAR_LEFT_DIR_PIN,
        REAR_RIGHT_STEP_PIN, REAR_RIGHT_DIR_PIN,
        L_X, L_Y, WHEEL_DIAMETER / 2);

    base->setMaxBaseSpeed(MAX_BASE_SPEED);
    base->setMaxBaseAngularSpeed(MAX_BASE_ANGULAR_SPEED);

    arm = new RoboticArm(NUM_JOINTS);
    arm->addJoint(JOINT_1_STEP_PIN, JOINT_1_DIR_PIN, -90, 90, 1, 17);
    arm->addJoint(JOINT_2_STEP_PIN, JOINT_2_DIR_PIN, -10, 90, 1, 19);
    arm->addJoint(JOINT_3_STEP_PIN, JOINT_3_DIR_PIN, -90, 90, 1, 18);

    arm->joints[0].reductionRatio = 5;                            // Pulley ratio
    arm->joints[1].reductionRatio = (5 + 2 / 11.0) * (60 / 36.0); // (Gear ratio stepper) * (Pulley ratio)
    arm->joints[2].reductionRatio = (5 + 2 / 11.0) * (60 / 36.0); // (Gear ratio stepper) * (Pulley ratio)

    arm->init();

    songPlayer = new SongPlayer(20);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    songPlayer->setSong(0);
    songPlayer->setRepeat(true);
}

void Robot::update() {
    float batteryPercentage = _getBatteryPercentage(A22);
    base->update();
    arm->update();
    songPlayer->update();
    if (batteryPercentage < 10) {
        LED('R');
        songPlayer->setSong(0);
    }
    // If joints are homing, set the LED to blue
    else if (arm->joints[0].homing || arm->joints[1].homing || arm->joints[2].homing)
        LED('Y');
    else
        LED('G');
}
void Robot::setSerial(Stream &serial) { this->serial = &serial; }

float Robot::_getBatteryPercentage(uint32_t pin) {
    static float res = 1024.0; // pow(2, 10);
    static float R1 = 10.07;
    static float R2 = 1.0;
    static float Vcc = 3.3;
    float Vout = Vcc * float(analogRead(pin)) / res;
    float Vs = Vout * (R1 + R2) / R2;
    float percent = map(Vs, 9.1, 12.6, 0, 100);

    return percent;
}

void Robot::LED(uint32_t r, uint32_t g, uint32_t b) {
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
}

void Robot::LED(char color) {
    switch (color) {
    case 'R':
    case 1:
        digitalWriteFast(RED_PIN, HIGH);
        digitalWriteFast(GREEN_PIN, LOW);
        digitalWriteFast(BLUE_PIN, LOW);
        break;
    case 'G':
    case 2:
        digitalWriteFast(RED_PIN, LOW);
        digitalWriteFast(GREEN_PIN, HIGH);
        digitalWriteFast(BLUE_PIN, LOW);
        break;
    case 'B':
    case 3:
        digitalWriteFast(RED_PIN, LOW);
        digitalWriteFast(GREEN_PIN, LOW);
        digitalWriteFast(BLUE_PIN, HIGH);
        break;
    case 'C':
    case 4:
        digitalWriteFast(RED_PIN, LOW);
        digitalWriteFast(GREEN_PIN, HIGH);
        digitalWriteFast(BLUE_PIN, HIGH);
        break;
    case 'M':
    case 5:
        digitalWriteFast(RED_PIN, HIGH);
        digitalWriteFast(GREEN_PIN, LOW);
        digitalWriteFast(BLUE_PIN, HIGH);
        break;
    case 'Y':
    case 6:
        digitalWriteFast(RED_PIN, HIGH);
        digitalWriteFast(GREEN_PIN, HIGH);
        digitalWriteFast(BLUE_PIN, LOW);
        break;
    case 'W':
    case 7:
        digitalWriteFast(RED_PIN, HIGH);
        digitalWriteFast(GREEN_PIN, HIGH);
        digitalWriteFast(BLUE_PIN, HIGH);
        break;
    case 'K':
    case 0:
        digitalWriteFast(RED_PIN, LOW);
        digitalWriteFast(GREEN_PIN, LOW);
        digitalWriteFast(BLUE_PIN, LOW);
        break;
    }
}