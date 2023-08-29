#include "RoboticArm.h"
#define pr(x) Serial.print(x)
#define prn(x) Serial.println(x)
RoboticArm::RoboticArm(uint32_t numJoints) {
    _numJoints = numJoints;
    joints = new Joint[_numJoints];
    links = new Link[_numJoints];
}

void RoboticArm::init() {
    _controlMode = INITIAL_CONFIG;
    gripper.attach(GRIPPER_PIN);
}

void RoboticArm::update() {
    _calculateAngles();

    // Sweep the gripper
    gripper.write(val);

    val_prev = val;
    // 160 acik
    // 35 kapali

    switch (_controlMode) {
    case VELOCITY_IK: {
        bool success = _calculateInverseJacobian();
        if (success) {
            Matrix<3, 1> desiredJointVelocity = J_inv * desiredCartesianVelocity; // rad/s
            // Serial.print(desiredJointVelocity(0, 0));
            // Serial.print(" ");
            // Serial.print(desiredJointVelocity(1, 0));
            // Serial.print(" ");
            // Serial.println(desiredJointVelocity(2, 0));
            float jointVelocity1 = desiredJointVelocity(0, 0) * 180 / PI * joints[0].reductionRatio * joints[0].microStep * joints[0].stepsPerRev / 360;
            float jointVelocity2 = desiredJointVelocity(1, 0) * 180 / PI * joints[1].reductionRatio * joints[1].microStep * joints[1].stepsPerRev / 360;
            float jointVelocity3 = desiredJointVelocity(2, 0) * 180 / PI * joints[2].reductionRatio * joints[2].microStep * joints[2].stepsPerRev / 360;
            
            // If 

            joints[0].motor.setSpeed(jointVelocity1);
            joints[0].motor.runSpeed();

            joints[1].motor.setSpeed(jointVelocity2);
            joints[1].motor.runSpeed();

            joints[2].motor.setSpeed(jointVelocity3);
            joints[2].motor.runSpeed();

            // 
        } else {
            // prn("Jacobian is not invertible");
        }

    } break;
    // Be careful with the hard coded values here, they are specific to the arm.
    case INITIAL_CONFIG: {

        float target;
        if (!joints[0].homed)
            homeJoint(joints[0]);
        else {
            target = 850 + INITAL_THETA_1 * joints[0].reductionRatio * joints[0].microStep * joints[0].stepsPerRev / 360;
            joints[0].motor.moveTo(target);
            joints[0].motor.run();
        }

        if (!joints[1].homed && joints[0].homed)
            homeJoint(joints[1]);
        // If joint 1 is homed, move it to the correct position
        if (joints[1].homed) {
            target = (24 + INITAL_THETA_2) * joints[1].reductionRatio * joints[1].microStep * joints[1].stepsPerRev / 360;
            joints[1].motor.moveTo(target);
            joints[1].motor.run();
        }
        if (!joints[2].homed && joints[1].homed && joints[1].motor.distanceToGo() == 0)
            homeJoint(joints[2]);

        // If joint 2 is not homed or homing, move it with joint 1
        if (!joints[2].homed && !joints[2].homing && joints[1].motor.speed() != 0) {
            // Move with joint 1
            joints[2].motor.setSpeed(joints[1].motor.speed());
            joints[2].motor.runSpeed();
        }

        if (joints[2].homed && joints[1].motor.distanceToGo() == 0) {
            target = (15 + (90 + INITAL_THETA_3)) * joints[2].reductionRatio * joints[2].microStep * joints[2].stepsPerRev / 360;
            joints[2].motor.moveTo(target);
            joints[2].motor.run();
        }

        // If they all reach their target, change the mode
        if (joints[0].homed && joints[1].homed && joints[2].homed &&
            joints[0].motor.distanceToGo() == 0 && joints[1].motor.distanceToGo() == 0 && joints[2].motor.distanceToGo() == 0) {
            _controlMode = VELOCITY_IK;
            float initialAngle = INITAL_THETA_1;
            joints[0].motor.setCurrentPosition(initialAngle);

            initialAngle = INITAL_THETA_2 * joints[1].reductionRatio * joints[1].microStep * joints[1].stepsPerRev / 360;
            joints[1].motor.setCurrentPosition(initialAngle);

            initialAngle = 0;
            joints[2].motor.setCurrentPosition(initialAngle);

            for (uint32_t i = 0; i < _numJoints; i++) {
                joints[i].homed = false;
            }
        }

        break;
    }

    default:
        break;
    }

    for (uint32_t i = 0; i < _numJoints; i++) {
        _readJointSwitch(joints[i]);
        _homeJoint(joints[i]);
        joints[i].switchStatePrev = joints[i].switchState;
    }
}

void RoboticArm::addJoint(uint32_t stepPin, uint32_t dirPin, int32_t angleLimitLower, int32_t angleLimitUpper, float reductionRatio, uint32_t switchPin) {
    Joint joint;
    joint.motor = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    joint.motor.setMaxSpeed(10000);
    joint.motor.setAcceleration(100000);
    joint.lowerAngleLimit = angleLimitLower;
    joint.upperAngleLimit = angleLimitUpper;
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

void RoboticArm::homeJoint(Joint &joint, bool waitForPreviousJoints = false) {
    if (joint.homing)
        return;
    // Assign joint homing priority
    if (waitForPreviousJoints) {
        // 0 is highest priority
        uint32_t highestPriority = 0;
        for (uint32_t i = 0; i < _numJoints; i++) {
            if (i == joint.ID) {
                continue;
            }
            if (joints[i].homing && joints[i].homingPriority > highestPriority)
                highestPriority = joints[i].homingPriority;
        }
        joint.homingPriority = highestPriority + 1;
    } else {
        joint.homingPriority = 0;
    }
    joint.homing = true;
    joint.homingState = Joint::HomingState::MOVING_TO_LIMIT_SWITCH;
}

void RoboticArm::_homeJoint(Joint &joint) {
    for (uint32_t i = 0; i < _numJoints; i++) {
        if (joints[i].homing && joints[i].homingPriority < joint.homingPriority)
            return;
    }
    switch (joint.homingState) {
    case Joint::HomingState::IDLE:
        joint.homing = false;
        joint.motor.setMaxSpeed(8000);
        joint.motor.setAcceleration(4000);
        break;

    case Joint::HomingState::MOVING_TO_LIMIT_SWITCH:
        if (joint.switchState) {
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
        if (joint.switchState == LOW) {
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

void RoboticArm::_readJointSwitch(Joint &joint) {

    joint.switchReading = digitalRead(joint.switchPin);
    if (joint.switchReading != joint.switchReadingPrev)
        joint.switchDebounceTime = millis();
    if ((millis() - joint.switchDebounceTime) > 30)
        joint.switchState = joint.switchReading;
    joint.switchReadingPrev = joint.switchReading;
}

void RoboticArm::homeAllJoints() {
    for (uint32_t i = 0; i < _numJoints; i++) {
        homeJoint(joints[i], i == 2 ? true : false);
    }
}

void RoboticArm::_calculateAngles() {
    joints[0].motorAngle = joints[0].motor.currentPosition() / joints[0].reductionRatio / joints[0].microStep / joints[0].stepsPerRev * 360;
    joints[0].jointAngle = joints[0].motorAngle;

    joints[1].motorAngle = joints[1].motor.currentPosition() / joints[1].reductionRatio / joints[1].microStep / joints[1].stepsPerRev * 360;
    joints[1].jointAngle = joints[1].motorAngle;

    joints[2].motorAngle = joints[2].motor.currentPosition() / joints[2].reductionRatio / joints[2].microStep / joints[2].stepsPerRev * 360;
    joints[2].jointAngle = joints[2].motorAngle - joints[1].jointAngle;

    // Serial.print(joints[0].jointAngle);
    // Serial.print(" ");
    // Serial.print(joints[1].jointAngle);
    // Serial.print(" ");
    // Serial.println(joints[2].jointAngle);
}
bool RoboticArm::_calculateInverseJacobian() {
    float alpha1 = joints[0].jointAngle;
    float alpha2 = joints[1].jointAngle;
    float alpha3 = joints[2].jointAngle;

    // J_inv(0, 0) = (sin((alpha1 * PI) / 1.8E+2) * -1.8E+3) / (PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.811E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 1.943E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.943E+3 + PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) * 1.811E+3);
    // J_inv(0, 1) = (cos((alpha1 * PI) / 1.8E+2) * 1.8E+3) / (PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.811E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 1.943E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.943E+3 + PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) * 1.811E+3);
    // J_inv(0, 2) = 0;
    // J_inv(1, 0) = (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha1 * PI) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    // J_inv(1, 1) = (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha1 * PI) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    // J_inv(1, 2) = (sin((PI * (alpha2 + alpha3)) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2));
    // J_inv(2, 0) = (cos((alpha1 * PI) / 1.8E+2) * (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.943E+3 + cos((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.115419494238475E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    // J_inv(2, 1) = (sin((alpha1 * PI) / 1.8E+2) * (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.943E+3 + cos((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.115419494238475E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    // J_inv(2, 2) = ((sin((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.943E+3 + sin((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.115419494238475E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2));

    J_inv(0, 0) = (sin((alpha1 * PI) / 1.8E+2) * -1.8E+3) / (PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.811E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 1.773E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.773E+3 + PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) * 1.811E+3);
    J_inv(0, 1) = (cos((alpha1 * PI) / 1.8E+2) * 1.8E+3) / (PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 4.64E+2 + PI * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.811E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * 1.773E+3 + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * 1.773E+3 + PI * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) * 1.811E+3);
    J_inv(0, 2) = 0;
    J_inv(1, 0) = (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha1 * PI) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    J_inv(1, 1) = (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha1 * PI) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    J_inv(1, 2) = (sin((PI * (alpha2 + alpha3)) / 1.8E+2) * (-9.939260077305356E-1)) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2));
    J_inv(2, 0) = (cos((alpha1 * PI) / 1.8E+2) * (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.773E+3 + cos((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.605899648790387E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    J_inv(2, 1) = (sin((alpha1 * PI) / 1.8E+2) * (cos((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.773E+3 + cos((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.605899648790387E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(cos((alpha1 * PI) / 1.8E+2), 2.0) * cos((alpha2 * PI) / 1.8E+2) + PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2) * pow(sin((alpha1 * PI) / 1.8E+2), 2.0));
    J_inv(2, 2) = ((sin((PI * (alpha2 + alpha3)) / 1.8E+2) * 1.773E+3 + sin((alpha2 * PI) / 1.8E+2) * 1.811E+3) * 5.605899648790387E-4) / (PI * cos((PI * (alpha2 + alpha3)) / 1.8E+2) * sin((alpha2 * PI) / 1.8E+2) - PI * sin((PI * (alpha2 + alpha3)) / 1.8E+2) * cos((alpha2 * PI) / 1.8E+2));

    // Iterate all elements in the matrix and check if they are NaN or Inf
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            if (isnan(J_inv(i, j)) || isinf(J_inv(i, j))) {
                return false;
            }
        }
    }

    return true;
}