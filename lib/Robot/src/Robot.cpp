#include "Robot.h"

Robot::Robot()
{
    base = new MobileBase(
        FRONT_LEFT_STEP_PIN, FRONT_LEFT_DIR_PIN, 
        FRONT_RIGHT_STEP_PIN, FRONT_RIGHT_DIR_PIN, 
        REAR_LEFT_STEP_PIN, REAR_LEFT_DIR_PIN, 
        REAR_RIGHT_STEP_PIN, REAR_RIGHT_DIR_PIN, 
        L_X, L_Y, WHEEL_DIAMETER / 2);

    base->setMaxBaseSpeed(MAX_BASE_SPEED);
    base->setMaxBaseAngularSpeed(MAX_BASE_ANGULAR_SPEED);

    arm = new RoboticArm(NUM_JOINTS);
    arm->addJoint(JOINT_1_STEP_PIN, JOINT_1_DIR_PIN, 0, 1, 17);
    arm->addJoint(JOINT_2_STEP_PIN, JOINT_2_DIR_PIN, 0, 1, 18);
    arm->addJoint(JOINT_3_STEP_PIN, JOINT_3_DIR_PIN, 0, 1, 19);

    arm->homeJoint(arm->joints[0]);
    arm->homeJoint(arm->joints[1]);
    arm->homeJoint(arm->joints[2]);
}

void Robot::update()
{
    base->update();
    arm->update();
}
void Robot::setSerial(Stream &serial) { this->serial = &serial; }