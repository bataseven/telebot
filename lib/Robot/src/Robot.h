#ifndef Robot_H
#define Robot_H
#include "Arduino.h"
#include "MobileBase.h"
#include "RoboticArm.h"
#include "SongPlayer.h"

#define FRONT_LEFT_STEP_PIN 4
#define FRONT_LEFT_DIR_PIN 6
#define FRONT_RIGHT_STEP_PIN 2
#define FRONT_RIGHT_DIR_PIN 3
#define REAR_LEFT_STEP_PIN 11
#define REAR_LEFT_DIR_PIN 12
#define REAR_RIGHT_STEP_PIN 24
#define REAR_RIGHT_DIR_PIN 25

#define NUM_JOINTS 3
#define JOINT_1_STEP_PIN 31
#define JOINT_1_DIR_PIN 32
#define JOINT_2_STEP_PIN 35
#define JOINT_2_DIR_PIN 36
#define JOINT_3_STEP_PIN 37
#define JOINT_3_DIR_PIN 38

#define WHEEL_DIAMETER .080 // m
#define L_X 0.1275          // m
#define L_Y 0.245 / 2       // m

#define MAX_BASE_SPEED 0.5        // m/s
#define MAX_BASE_ANGULAR_SPEED PI // rad/s

#define RED_PIN 14
#define GREEN_PIN 15
#define BLUE_PIN 16

class Robot {
private:
    Stream *serial;
    float _getBatteryPercentage(uint32_t pin);
    void LED(uint32_t r, uint32_t g, uint32_t b);
    void LED(char color);

public:
    Robot();
    void update();
    MobileBase *base;
    RoboticArm *arm;
    SongPlayer *songPlayer;
    void setSerial(Stream &serial);
    float batteryPercentage;
};

#endif