#ifndef RoboticArm_H
#define RoboticArm_H
#include "AccelStepper.h"
#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Servo.h"
using namespace BLA;

#define INITAL_THETA_1 0
#define INITAL_THETA_2 60
#define INITAL_THETA_3 -60

#define GRIPPER_PIN 33
struct Joint {
    int32_t ID;
    AccelStepper motor;
    int32_t lowerAngleLimit = 0;
    int32_t upperAngleLimit = 0;
    uint32_t microStep = 8;
    uint32_t stepsPerRev = 200;

    float motorAngle = 0;
    float jointAngle = 0;

    float reductionRatio;

    uint32_t switchPin;
    bool switchReading = false;
    bool switchReadingPrev = false;
    uint32_t switchDebounceTime = 0;
    bool switchState = false;
    bool switchStatePrev = false;

    bool homed = false;
    bool homing = false;
    enum HomingState { IDLE,
                       MOVING_TO_LIMIT_SWITCH,
                       MOVING_TO_ZERO };
    HomingState homingState = HomingState::IDLE;
    // 0 is highest priority
    uint32_t homingPriority = 0;
    int32_t homingStartPos = 0;
};

struct Link {
    uint32_t length = 0;
    uint32_t mass = 0;
};

class RoboticArm {
public:
    enum ControlMode {
        NONE,
        VELOCITY_IK,
        INITIAL_CONFIG
    };
    RoboticArm(uint32_t numJoints);
    void init();
    void update();
    void addJoint(uint32_t stepPin, uint32_t dirPin, int32_t angleLimitLower, int32_t angleLimitUpper, float reductionRatio, uint32_t switchPin);
    void setControlMode(ControlMode controlMode) { _controlMode = controlMode; }
    void homeJoint(Joint &joint, bool waitForPreviousJoints = false);
    void homeAllJoints();

    Servo gripper;
    Joint *joints;
    Link *links;
    Matrix<3> desiredCartesianVelocity = Zeros<3>(); // mm/s
    float val = 90;
    float val_prev = 90;

private:
    uint32_t _numJoints;
    int32_t _jointIndex = 0;
    ControlMode _controlMode = NONE;
    float inc = 0.0005;
    void _homeJoint(Joint &joint);
    void _readJointSwitch(Joint &joint);
    void _calculateAngles();
    bool _calculateInverseJacobian();
    Matrix<3, 3> J = Zeros<3, 3>();
    Matrix<3, 3> J_inv = Zeros<3, 3>();
};

#endif