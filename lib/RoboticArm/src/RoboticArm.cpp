#include "RoboticArm.h"

RoboticArm::RoboticArm(uint32_t numJoints)
{
    _numJoints = numJoints;
    joints = new Joint[_numJoints];
    links = new Link[_numJoints];
}

void RoboticArm::update()
{
    _calculateAngles();

    switch (_controlMode)
    {
    case VELOCITY_IK:
        Serial.println("VELOCITY_IK");
        break;
    // Be careful with the hard coded values here, they are specific to the arm.
    case INITIAL_CONFIG:
        if (!joints[0].homed)
            homeJoint(joints[0]);
        else
        {
            joints[0].motor.moveTo(850);
            joints[0].motor.run();
        }

        if (!joints[1].homed && joints[0].homed)
            homeJoint(joints[1]);
        // If joint 1 is homed, move it to the correct position
        if (joints[1].homed)
        {
            uint32_t target = (24 + 90) * joints[1].reductionRatio * joints[1].microStep * joints[1].stepsPerRev / 360;
            joints[1].motor.moveTo(target);
            joints[1].motor.run();
        }
        if (!joints[2].homed && joints[1].homed && joints[1].motor.distanceToGo() == 0)
            homeJoint(joints[2]);

        // If joint 2 is not homed or homing, move it with joint 1
        if (!joints[2].homed && !joints[2].homing && joints[1].motor.speed() != 0)
        {
            // Move with joint 1
            joints[2].motor.setSpeed(joints[1].motor.speed());
            joints[2].motor.runSpeed();
        }

        if (joints[2].homed && joints[1].motor.distanceToGo() == 0)
        {
            float target = 13.5 * joints[2].reductionRatio * joints[2].microStep * joints[2].stepsPerRev / 360;
            joints[2].motor.moveTo(target);
            joints[2].motor.run();
        }

        // If they all reach their target, change the mode
        if (joints[0].homed && joints[1].homed && joints[2].homed &&
            joints[0].motor.distanceToGo() == 0 && joints[1].motor.distanceToGo() == 0 && joints[2].motor.distanceToGo() == 0)
        {
            _controlMode = VELOCITY_IK;
            float initialAngle = 0;
            joints[0].motor.setCurrentPosition(initialAngle);

            initialAngle = 90 * joints[1].reductionRatio * joints[1].microStep * joints[1].stepsPerRev / 360;
            joints[1].motor.setCurrentPosition(initialAngle);

            initialAngle = 0;
            joints[2].motor.setCurrentPosition(initialAngle);
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

void RoboticArm::init()
{
    _controlMode = INITIAL_CONFIG;
}

void RoboticArm::addJoint(uint32_t stepPin, uint32_t dirPin, uint32_t jointLimit, float reductionRatio, uint32_t switchPin)
{
    Joint joint;
    joint.motor = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    joint.motor.setMaxSpeed(10000);
    joint.motor.setAcceleration(100000);
    joint.limit = jointLimit;
    joint.reductionRatio = reductionRatio;
    joint.switchPin = switchPin;
    joints[_jointIndex] = joint;

    Link link;
    link.length = 0;
    link.mass = 0;
    links[_jointIndex] = link;

    joint.ID = _jointIndex;
    _jointIndex++;

    pinMode(switchPin, INPUT);
}

void RoboticArm::homeJoint(Joint &joint, bool waitForPreviousJoints = false)
{
    if (joint.homing)
        return;
    // Assign joint homing priority
    if (waitForPreviousJoints)
    {
        // 0 is highest priority
        uint32_t highestPriority = 0;
        for (uint32_t i = 0; i < _numJoints; i++)
        {
            if (i == joint.ID)
            {
                continue;
            }
            if (joints[i].homing && joints[i].homingPriority > highestPriority)
                highestPriority = joints[i].homingPriority;
        }
        joint.homingPriority = highestPriority + 1;
    }
    else
    {
        joint.homingPriority = 0;
    }
    joint.homing = true;
    joint.homingState = Joint::HomingState::MOVING_TO_LIMIT_SWITCH;
}

void RoboticArm::_homeJoint(Joint &joint)
{
    for (uint32_t i = 0; i < _numJoints; i++)
    {
        if (joints[i].homing && joints[i].homingPriority < joint.homingPriority)
            return;
    }
    switch (joint.homingState)
    {
    case Joint::HomingState::IDLE:
        joint.homing = false;
        joint.motor.setMaxSpeed(5000);
        joint.motor.setAcceleration(2000);
        break;

    case Joint::HomingState::MOVING_TO_LIMIT_SWITCH:
        if (joint.switchState)
        {
            joint.homingStartPos = -joint.motor.currentPosition();
            joint.motor.setSpeed(0);
            joint.motor.runSpeed();
            joint.homingState = Joint::HomingState::MOVING_TO_ZERO;
            break;
        }
        joint.motor.setAcceleration(100);
        joint.motor.setSpeed(-600);
        joint.motor.runSpeed();
        break;

    case Joint::HomingState::MOVING_TO_ZERO:
        if (joint.switchState == LOW)
        {
            joint.motor.setCurrentPosition(0);
            joint.motor.setSpeed(0);
            joint.motor.runSpeed();
            joint.homingState = Joint::HomingState::IDLE;
            joint.homed = true;
            break;
        }
        joint.motor.setSpeed(150);
        joint.motor.runSpeed();
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

void RoboticArm::homeAllJoints()
{
    for (uint32_t i = 0; i < _numJoints; i++)
    {
        homeJoint(joints[i], i == 2 ? true : false);
    }
}

void RoboticArm::_calculateAngles()
{
    joints[0].motorAngle = joints[0].motor.currentPosition() / joints[0].reductionRatio / joints[0].microStep / joints[0].stepsPerRev * 360;
    joints[0].jointAngle = joints[0].motorAngle;

    joints[1].motorAngle = joints[1].motor.currentPosition() / joints[1].reductionRatio / joints[1].microStep / joints[1].stepsPerRev * 360;
    joints[1].jointAngle = joints[1].motorAngle;

    joints[2].motorAngle = joints[2].motor.currentPosition() / joints[2].reductionRatio / joints[2].microStep / joints[2].stepsPerRev * 360;
    joints[2].jointAngle = joints[2].motorAngle - joints[1].jointAngle;

    Serial.print(joints[0].jointAngle);
    Serial.print(" ");
    Serial.print(joints[1].jointAngle);
    Serial.print(" ");
    Serial.println(joints[2].jointAngle);
}