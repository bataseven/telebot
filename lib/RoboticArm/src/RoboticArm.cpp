#include "RoboticArm.h"

RoboticArm::RoboticArm(uint32_t numJoints)
{
    _numJoints = numJoints;
    joints = new Joint[_numJoints];
    links = new Link[_numJoints];
}

void RoboticArm::update()
{

    switch (_controlMode)
    {
    case VELOCITY_IK:
        for (uint32_t i = 0; i < _numJoints; i++)
        {
            if (joints[i].homing)
                continue;
            joints[i].motor.runSpeed();
        }
        break;

    default:
        break;
    }

    for (uint32_t i = 0; i < _numJoints; i++)
    {
        _readJointSwitch(joints[i]);
        _homeJoint(joints[i]);
        joints[i].switchStatePrev = joints[i].switchState;
    }
}

void RoboticArm::addJoint(uint32_t stepPin, uint32_t dirPin, uint32_t jointLimit, float reductionRatio, uint32_t switchPin)
{
    Joint joint;
    joint.motor = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    joint.limit = jointLimit;
    joint.reductionRatio = reductionRatio;
    joint.switchPin = switchPin;
    joints[_jointIndex] = joint;

    Link link;
    link.length = 0;
    link.mass = 0;
    links[_jointIndex] = link;

    _jointIndex++;

    pinMode(switchPin, INPUT);
}

void RoboticArm::homeJoint(Joint &joint)
{
    if (joint.homing)
        return;
    joint.homing = true;
    joint.homingState = Joint::HomingState::MOVING_TO_LIMIT_SWITCH;
}

void RoboticArm::_homeJoint(Joint &joint)
{
    switch (joint.homingState)
    {
    case Joint::HomingState::IDLE:
        joint.homing = false;
        break;

    case Joint::HomingState::MOVING_TO_LIMIT_SWITCH:
        if (joint.switchState)
        {
            joint.motor.setCurrentPosition(0);
            joint.motor.setSpeed(0);
            joint.homingState = Joint::HomingState::MOVING_TO_ZERO;
            break;
        }
        joint.motor.setSpeed(-100);
        joint.motor.runSpeed();
        break;

    case Joint::HomingState::MOVING_TO_ZERO:
        if (joint.switchState == LOW)
        {
            joint.motor.setSpeed(0);
            joint.homingState = Joint::HomingState::IDLE;
            joint.homed = true;
            break;
        }
        joint.motor.setSpeed(100);
        break;
    }
}

void RoboticArm::_readJointSwitch(Joint &joint)
{
    joint.switchReading = digitalRead(joint.switchPin);
    if (joint.switchReading != joint.switchReadingPrev)
        joint.switchDebounceTime = millis();
    if ((millis() - joint.switchDebounceTime) > 30)
        joint.switchState = joint.switchReading;
    joint.switchReadingPrev = joint.switchReading;
}