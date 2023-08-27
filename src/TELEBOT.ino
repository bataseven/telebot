// #include <AccelStepper.h>
// #include <Metro.h>
// #include "Melodies.h"
// #define XBeePort Serial1  // XBee Serial Port
// #define XBeeBaudrate 19200

// // #define JOYSTICK_STEER
// #define HOME_AT_STARTUP
// // #define DONT_HOME_1_AND_2
// #define DONT_HOME_2
// /////////////////////////////////////////////////////////////////
// #define WHEEL_MICRO_STEPPING 8
// #define MICRO_STEPPING_ARM 8
// #define ROBOT_STEPPER_MAX_POSITION_0 485 * MICRO_STEPPING_ARM
// #define ROBOT_STEPPER_MAX_POSITION_1 530 * MICRO_STEPPING_ARM
// #define ROBOT_STEPPER_MAX_POSITION_2 1030 * MICRO_STEPPING_ARM
// #define HOMING_SPEED 145 * MICRO_STEPPING_ARM
// #define HOMING_ACCELERATION 14.5 * MICRO_STEPPING_ARM
// #define WHEEL_STEPPER_MAX_SPEED 375 * WHEEL_MICRO_STEPPING  // 375 normal
// /////////////////////////////////////////////////////////////////
// #define CONNECTION_TIMEOUT_IN_MS 750
// /////////////////////////////////////////////////////////////////
// #define RED_PIN 14
// #define GREEN_PIN 15
// #define BLUE_PIN 16
// #define END_SWITCH_0 17
// #define END_SWITCH_1 18
// #define END_SWITCH_2 19
// #define BUZZER_PIN 20
// #define GRIPPER_OPEN 22
// #define GRIPPER_CLOSE 23
// /////////////////////////////////////////////////////////////////

// AccelStepper motor1(1, 2, 3);    // Type:driver,step,direction
// AccelStepper motor2(1, 4, 6);    // Type:driver,step,direction
// AccelStepper motor3(1, 11, 12);  // Type:driver,step,direction
// AccelStepper motor4(1, 24, 25);  // Type:driver,step,direction

// AccelStepper WHEEL_STEPPER_MOTORS[4] = {motor1, motor2, motor3, motor4};

// AccelStepper motor5(1, 31, 32);  // Type:driver,step,direction
// AccelStepper motor6(1, 35, 36);  // Type:driver,step,direction
// AccelStepper motor7(1, 37, 38);  // Type:driver,step,direction

// AccelStepper ROBOT_STEPPER_MOTORS[3] = {motor5, motor6, motor7};

// int ROBOT_STEPPER_MAX_SPEED_DEFAULT = 630 * MICRO_STEPPING_ARM;
// int ROBOT_STEPPER_MAX_ACCELERATION_DEFAULT = 1300 * MICRO_STEPPING_ARM;

// const int RECORD_COUNT_MAX = 5000;
// int RECORD_INDEX = 0;
// int PLAY_INDEX = 0;
// int RECORD_COUNT = 0;

// char WORKING_MODE = 'M';  // M for manual, R for recoding, P for repeating the recorded pattern, C for otomatik, S for
//                           // stop actuating motors
// char WORKING_MODE_PREVIOUS = 'M';
// float RECEIVED_ANGLES[2];

// float ROBOT_STEPS[3] = {ROBOT_STEPPER_MAX_POSITION_0 / 2, 0, 0};
// long ROBOT_STEPS_RECORDED[3][RECORD_COUNT_MAX];

// float WHEEL_SPEEDS[4];
// float WHEEL_STEPS[4];
// long WHEEL_STEPS_RECORDED[4][RECORD_COUNT_MAX];
// int WHEEL_SPEEDS_RECORDED[4][RECORD_COUNT_MAX];

// float gearRatio = 5 + 2 / 11.0;
// float beltRatio = 60 / 36.0;
// float beltRatio2 = 100 / 20.0;
// int stepPerRev = 200;    

// float Battery_Percent;
// float Battery_Voltage;

// int dSTEP = 1;

// const byte numChars = 64;
// char receivedChars[numChars];
// boolean newData = true;  // Is set to true when a new valid data is received
// unsigned long newDataTimer;

// unsigned long printTimer = 0;
// unsigned long increaseTimer = 0;

// boolean DEVICE_DISCONNECTED;

// int Selected_Song = 0;

// Melodies melody(BUZZER_PIN);

// void setup() {
// #if XBeePort != Serial
//     Serial.begin(XBeeBaudrate);
// #endif
//     XBeePort.begin(XBeeBaudrate);
//     Serial.begin(XBeeBaudrate);
//     Serial.println("XBee Serial Port is initialized!");

//     pinMode(21, OUTPUT);
//     digitalWrite(21, LOW);

//     pinMode(END_SWITCH_0, INPUT);
//     pinMode(END_SWITCH_1, INPUT);
//     pinMode(END_SWITCH_2, INPUT);

//     pinMode(RED_PIN, OUTPUT);
//     pinMode(GREEN_PIN, OUTPUT);
//     pinMode(BLUE_PIN, OUTPUT);

//     pinMode(GRIPPER_OPEN, OUTPUT);
//     pinMode(GRIPPER_CLOSE, OUTPUT);

//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, HIGH);

//     while (!Serial)
//         ;
//     Serial.println("Serial Port is initialized!");
//     digitalWrite(LED_BUILTIN, LOW);

//     for (int i = 0; i < 4; i++) {
//         WHEEL_STEPPER_MOTORS[i].setMaxSpeed(WHEEL_STEPPER_MAX_SPEED);
//         WHEEL_STEPPER_MOTORS[i].setAcceleration(150000);
//     }

// Battery_Voltage = Calculate_Battery_Voltage(A22);  // Returns Voltage
// Battery_Percent = map(Battery_Voltage, 9.1, 12.6, 0, 100);

//     // while (Battery_Percent <= 0) {
//     //   if ((millis() / 500) % 2 == 0)  // Make the LED blink in red every half a second
//     //     LED('R');
//     //   else
//     //     LED('O');
//     //   Battery_Voltage = Calculate_Battery_Voltage(A22); // Returns Voltage
//     //   Battery_Percent = map(Battery_Voltage, 9.1, 12.6, 0, 100);
//     //   if (millis() - printTimer >= 30000) {
//     //     printTimer = millis();
//     //     Serial.print("Low battery voltage: ");
//     //     Serial.print(Battery_Voltage);
//     //     Serial.print("V (");
//     //     Serial.print(Battery_Percent);
//     //     Serial.println("%)");
//     //   }
//     //   // break;
//     // }

//     Battery_Voltage = Calculate_Battery_Voltage(A22);  // Returns Voltage
//     Battery_Percent = map(Battery_Voltage, 9.1, 12.6, 0, 100);
//     if (Battery_Percent <= 0) {
//         Serial.print("Low battery voltage: ");
//         Serial.print(Battery_Voltage);
//         Serial.print("V (");
//         Serial.print(Battery_Percent);
//         Serial.println("%)");
//     }

//     LED('Y');
//     /////// HOMING SEQUENCE //////
//     // #ifdef HOME_AT_STARTUP

//       // HOME_STEPPER(ROBOT_STEPPER_MOTORS[0], END_SWITCH_0);
//     //   ROBOT_STEPPER_MOTORS[0].runToNewPosition(ROBOT_STEPPER_MAX_POSITION_0 / 2);

//     // #ifndef DONT_HOME_1_AND_2

//     // #ifndef DONT_HOME_2
//     //   HOME_STEPPER(ROBOT_STEPPER_MOTORS[2], END_SWITCH_2);
//     //   ROBOT_STEPPER_MOTORS[2].runToNewPosition(450 * MICRO_STEPPING_ARM);
//     // #endif

//       // HOME_STEPPER(ROBOT_STEPPER_MOTORS[1], END_SWITCH_1);
//     //   ROBOT_STEPPER_MOTORS[1].runToNewPosition(ROBOT_STEPPER_MAX_POSITION_1);

//     // #ifndef DONT_HOME_2
//     //   HOME_STEPPER(ROBOT_STEPPER_MOTORS[2], END_SWITCH_2);
//     //   ROBOT_STEPPER_MOTORS[2].setCurrentPosition(ROBOT_STEPPER_MOTORS[1].currentPosition());
//     // #endif

//     // #endif

//   ROBOT_STEPS[0] =  stepPerRev * beltRatio2 * MICRO_STEPPING_ARM * RECEIVED_ANGLES[0] / 360 +
//   ROBOT_STEPPER_MAX_POSITION_0 / 2; ROBOT_STEPS[1] = ROBOT_STEPPER_MAX_POSITION_1;
//   ROBOT_STEPS[2] =
// stepPerRev * beltRatio  * gearRatio * MICRO_STEPPING_ARM * RECEIVED_ANGLES[1] / 360 + 5000; // Convert received angle to
//   steps

//     //   Serial.println("Homing completed for all!");

//     // #endif

//     Serial.println("Starting program");

// }

// void loop() {
//     if (millis() - printTimer > 1000) {
//         printTimer = millis();
//         if (WORKING_MODE == 'S' && Battery_Percent <= 0) {
//             Serial.print("Low battery voltage: ");
//             Serial.print(Battery_Voltage);
//             Serial.print("V (");
//             Serial.print(Selected_Song);
//             Serial.println("%)");
//         }
//         //    String msg = String(Battery_Percent) + "$";
//         //    XBeePort.print(msg);
//     }

//     recvWithStartEndMarkers();

//     Battery_Voltage = Calculate_Battery_Voltage(A22);  // Returns Voltage
//     Battery_Percent = map(Battery_Voltage, 9.1, 12.6, 0, 100);

//     handleLED();

//     if (millis() - newDataTimer > CONNECTION_TIMEOUT_IN_MS || Battery_Percent <= 0) {
//         WORKING_MODE = 'S';
//         GRIPPER_CONTROL(1);
//         if (Battery_Percent <= 0) Selected_Song = 3;
//     }

//     melody.play(Selected_Song);

//     if (WORKING_MODE == 'C') {
//         for (int i = 0; i < 3; i++) {
//             ROBOT_STEPPER_MOTORS[i].setAcceleration(ROBOT_STEPPER_MAX_ACCELERATION_DEFAULT * 2);
//             ROBOT_STEPPER_MOTORS[i].setMaxSpeed(ROBOT_STEPPER_MAX_SPEED_DEFAULT * 2);
//         }
//         if (newData) {
//             Serial.println(ROBOT_STEPPER_MOTORS[0].currentPosition());
//         }
//     }

//     if (newData ||
//         WORKING_MODE ==
//             'S') {  // Only enter if newData is available. That way robot follows the commands in discrete time steps.

//         switch (WORKING_MODE) {
//             case 'R':  //  RECORD
//                 ROBOT_STEPS[2] =
//                     stepPerRev * beltRatio * gearRatio * MICRO_STEPPING_ARM * RECEIVED_ANGLES[1] / 360 + 5000;
//                 ROBOT_STEPS[0] = -stepPerRev * beltRatio2 * MICRO_STEPPING_ARM * RECEIVED_ANGLES[0] / 360 +
//                                  ROBOT_STEPPER_MAX_POSITION_0 / 2;
//                 if (WORKING_MODE_PREVIOUS != 'R') {
//                     memset(WHEEL_STEPS_RECORDED, 0, sizeof(WHEEL_STEPS_RECORDED));
//                     memset(ROBOT_STEPS_RECORDED, 0, sizeof(ROBOT_STEPS_RECORDED));
//                     RECORD_COUNT = 0;
//                     RECORD_INDEX = 0;
//                     for (int i = 0; i < 4; i++) WHEEL_STEPPER_MOTORS[i].setCurrentPosition(0);
//                 }
//                 for (int i = 0; i < 3; i++)
//                     ROBOT_STEPS_RECORDED[i][RECORD_INDEX] = ROBOT_STEPPER_MOTORS[i].currentPosition();
//                 for (int i = 0; i < 4; i++)
//                     WHEEL_STEPS_RECORDED[i][RECORD_INDEX] = WHEEL_STEPPER_MOTORS[i].currentPosition();
//                 for (int i = 0; i < 4; i++) WHEEL_SPEEDS_RECORDED[i][RECORD_INDEX] = WHEEL_STEPPER_MOTORS[i].speed();
//                 RECORD_INDEX++;
//                 RECORD_COUNT++;
//                 if (RECORD_INDEX == RECORD_COUNT_MAX)
//                     WORKING_MODE = 'P';  // Automatically starts playing when the record array is full
//                 break;

//             case 'P':  //  PLAY
//                 if (RECORD_COUNT != 0) {
//                     if (WORKING_MODE_PREVIOUS != 'P') {
//                         PLAY_INDEX = 0;
//                         for (int i = 0; i < 4; i++)
//                             WHEEL_STEPPER_MOTORS[i].setCurrentPosition(WHEEL_STEPS_RECORDED[i][PLAY_INDEX]);
//                     }
//                     if (PLAY_INDEX == 0)
//                         for (int i = 0; i < 4; i++)
//                             WHEEL_STEPPER_MOTORS[i].setCurrentPosition(WHEEL_STEPS_RECORDED[i][PLAY_INDEX]);
//                     for (int i = 0; i < 4; i++) {
//                         WHEEL_STEPS[i] = WHEEL_STEPS_RECORDED[i][PLAY_INDEX];
//                         WHEEL_SPEEDS[i] = WHEEL_SPEEDS_RECORDED[i][PLAY_INDEX];
//                     }
//                     for (int i = 0; i < 3; i++) {
//                         //  if (i == 1)continue;  /// will be removed in the final edition
//                         ROBOT_STEPS[i] = ROBOT_STEPS_RECORDED[i][PLAY_INDEX];
//                     }
//                     PLAY_INDEX++;
//                     PLAY_INDEX %= RECORD_COUNT;
//                 }
//                 break;
//             case 'C':
//             case 'M':  //  MANUAL
//                 ROBOT_STEPS[2] = stepPerRev * beltRatio * gearRatio * MICRO_STEPPING_ARM * RECEIVED_ANGLES[1] / 360 +
//                                  5000;  // Convert received angle to steps
//                 ROBOT_STEPS[0] = -stepPerRev * beltRatio2 * MICRO_STEPPING_ARM * RECEIVED_ANGLES[0] / 360 +
//                                  ROBOT_STEPPER_MAX_POSITION_0 / 2;
//                 break;
//             case 'S':  // STOP
//                 for (int i = 0; i < 4; i++) WHEEL_STEPPER_MOTORS[i].setSpeed(0);
//                 break;
//         }
//     }
//     ROBOT_STEPS[0] = constrain(ROBOT_STEPS[0], 0, ROBOT_STEPPER_MAX_POSITION_0);
//     ROBOT_STEPS[1] = constrain(ROBOT_STEPS[1], 0, ROBOT_STEPPER_MAX_POSITION_1);
//     ROBOT_STEPS[2] = constrain(ROBOT_STEPS[2], ROBOT_STEPS[1], ROBOT_STEPS[1] + ROBOT_STEPPER_MAX_POSITION_2);

//     moveSteppers();

//     newData = false;

//     WORKING_MODE_PREVIOUS = WORKING_MODE;
// }
// void handleLED() {
//     if (Battery_Percent <= 0) {
//         if ((millis() / 500) % 2 == 0)  // Make the LED blink in red every half a second
//             LED('R');
//         else
//             LED('O');
//     } else if (Battery_Percent < 7)
//         LED('R');
//     else if (millis() - newDataTimer > CONNECTION_TIMEOUT_IN_MS)
//         LED('M');
//     else if (WORKING_MODE == 'C')
//         LED((millis() / 50) % 8);
//     else if (WORKING_MODE == 'R')
//         LED('B');
//     else if (WORKING_MODE == 'P')
//         LED('C');
//     else
//         LED('G');
// }

// void moveSteppers() {
//     for (int i = 0; i < 3; i++) {
//         ROBOT_STEPPER_MOTORS[i].moveTo(ROBOT_STEPS[i]);
//         ROBOT_STEPPER_MOTORS[i].run();
//     }
//     if (WORKING_MODE != 'P')
//         for (int i = 0; i < 4; i++) WHEEL_STEPPER_MOTORS[i].runSpeed();
//     else {
//         for (int i = 0; i < 4; i++) {
//             WHEEL_STEPPER_MOTORS[i].moveTo(WHEEL_STEPS[i]);
//             WHEEL_STEPPER_MOTORS[i].setSpeed(WHEEL_SPEEDS[i]);
//             WHEEL_STEPPER_MOTORS[i].runSpeedToPosition();
//         }
//     }
// }

// void recvWithStartEndMarkers() {
//     static boolean recvInProgress = false;
//     static byte ndx = 0;
//     char startMarker = '<';
//     char endMarker = '>';
//     char rc;

//     if (XBeePort.available() > 0) {
//         rc = XBeePort.read();
//         if (recvInProgress == true) {
//             if (rc != endMarker) {
//                 receivedChars[ndx] = rc;
//                 ndx++;
//                 if (ndx >= numChars) {
//                     Serial.println("Buffer Overflow");
//                     ndx = numChars - 1;
//                 }
//             } else {
//                 digitalWrite(13, !digitalRead(13));
//                 receivedChars[ndx] = '\0';  // terminate the string when the end marker arrives
//                 recvInProgress = false;
//                 ndx = 0;
//                 newData = true;
//                 newDataTimer = millis();
//                 parseData(receivedChars);
//             }
//         }

//         else if (rc == startMarker) {
//             recvInProgress = true;
//         }
//     }
// }

// void parseData(String Data) {
//     float H100 = ((int)Data[14] - '0') * 100;
//     float H10 = ((int)Data[15] - '0') * 10;
//     float H1 = ((int)Data[16] - '0');
//     float H01 = ((int)Data[17] - '0') * 0.1;
//     float H001 = ((int)Data[18] - '0') * 0.01;

//     float P100 = ((int)Data[20] - '0') * 100;
//     float P10 = ((int)Data[21] - '0') * 10;
//     float P1 = ((int)Data[22] - '0');
//     float P01 = ((int)Data[23] - '0') * 0.1;
//     float P001 = ((int)Data[24] - '0') * 0.01;

//     RECEIVED_ANGLES[1] = (Data[19] == '-') ? ((P100 + P10 + P1 + P01 + P001) * -1) : (P100 + P10 + P1 + P01 + P001);
//     RECEIVED_ANGLES[0] = (Data[13] == '-') ? ((H100 + H10 + H1 + H01 + H001) * -1) : (H100 + H10 + H1 + H01 + H001);

//     GRIPPER_CONTROL(Data[12] - '0');

//     Selected_Song = Data[6] - '0';

//     WORKING_MODE = Data[25];

//     setMotorSpeed(Data);
// }

// void setMotorSpeed(String Data) {
//     int xVel = (Data[1] - '0') * 10 + (Data[2] - '0');
//     int yVel = (Data[4] - '0') * 10 + (Data[5] - '0');

//     float Velocity_Vector_Angle;
//     float Velocity_Vector_Mag;

//     if (Data[0] == '-') xVel = -xVel;

//     if (Data[3] == '-') yVel = -yVel;

// #ifndef JOYSTICK_STEER

//     xVel *= -1;

//     if (yVel != 0 || xVel != 0) {
//         Velocity_Vector_Angle = 180 - (degrees(atan2(yVel, xVel)) - RECEIVED_ANGLES[0]);
//         Velocity_Vector_Mag = constrain(sqrt(yVel * yVel + xVel * xVel), 0, 40);

//         if (Velocity_Vector_Angle < 0) Velocity_Vector_Angle += 360;

//         float X = Velocity_Vector_Mag / 40;

//         if (Velocity_Vector_Angle >= 0 && Velocity_Vector_Angle <= 90) {
//             WHEEL_SPEEDS[0] = WHEEL_STEPPER_MAX_SPEED * X;
//             WHEEL_SPEEDS[2] = -WHEEL_SPEEDS[0];
//             WHEEL_SPEEDS[1] =
//                 -map(Velocity_Vector_Angle, 0, 90, -WHEEL_STEPPER_MAX_SPEED * X, WHEEL_STEPPER_MAX_SPEED * X);
//             WHEEL_SPEEDS[3] = -WHEEL_SPEEDS[1];
//         } else if (Velocity_Vector_Angle > 90 && Velocity_Vector_Angle <= 180) {
//             WHEEL_SPEEDS[0] =
//                 map(Velocity_Vector_Angle, 90, 180, WHEEL_STEPPER_MAX_SPEED * X, -WHEEL_STEPPER_MAX_SPEED * X);
//             WHEEL_SPEEDS[2] = -WHEEL_SPEEDS[0];
//             WHEEL_SPEEDS[1] = -WHEEL_STEPPER_MAX_SPEED * X;
//             WHEEL_SPEEDS[3] = -WHEEL_SPEEDS[1];
//         } else if (Velocity_Vector_Angle > 180 && Velocity_Vector_Angle <= 270) {
//             WHEEL_SPEEDS[0] = -WHEEL_STEPPER_MAX_SPEED * X;
//             WHEEL_SPEEDS[2] = -WHEEL_SPEEDS[0];
//             WHEEL_SPEEDS[1] =
//                 -map(Velocity_Vector_Angle, 180, 270, WHEEL_STEPPER_MAX_SPEED * X, -WHEEL_STEPPER_MAX_SPEED * X);
//             WHEEL_SPEEDS[3] = -WHEEL_SPEEDS[1];
//         } else {
//             WHEEL_SPEEDS[0] =
//                 map(Velocity_Vector_Angle, 270, 360, -WHEEL_STEPPER_MAX_SPEED * X, WHEEL_STEPPER_MAX_SPEED * X);
//             WHEEL_SPEEDS[2] = -WHEEL_SPEEDS[0];
//             WHEEL_SPEEDS[1] = WHEEL_STEPPER_MAX_SPEED * X;
//             WHEEL_SPEEDS[3] = -WHEEL_SPEEDS[1];
//         }
//     } else {
//         for (int i = 0; i < 4; i++) WHEEL_SPEEDS[i] = 0;
//     }

//     int rotation1 = Data[10] - '0';  // ButtonF
//     int rotation2 = Data[11] - '0';  // ButtonE

//     if (rotation1 == 0 || rotation2 == 0) {
//         int divider = 2;
//         if (WORKING_MODE == 'C') divider = 5;
//         if (rotation1 == 1) {
//             WHEEL_SPEEDS[0] -= WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[1] -= WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[2] -= WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[3] -= WHEEL_STEPPER_MAX_SPEED / divider;
//         }
//         if (rotation2 == 1) {
//             WHEEL_SPEEDS[0] += WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[1] += WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[2] += WHEEL_STEPPER_MAX_SPEED / divider;
//             WHEEL_SPEEDS[3] += WHEEL_STEPPER_MAX_SPEED / divider;
//         }
//     }
// #else
//     Velocity_Vector_Mag = constrain(sqrt(yVel * yVel + xVel * xVel), 0, 9);
//     WHEEL_SPEEDS[0] = map(yVel, -9, 9, -WHEEL_STEPPER_MAX_SPEED * 0.9, WHEEL_STEPPER_MAX_SPEED * 0.9);
//     WHEEL_SPEEDS[3] = WHEEL_SPEEDS[0];
//     WHEEL_SPEEDS[2] = -WHEEL_SPEEDS[0];
//     WHEEL_SPEEDS[1] = WHEEL_SPEEDS[2];

//     int steer = map(xVel, -9, 9, -WHEEL_STEPPER_MAX_SPEED * 0.55, WHEEL_STEPPER_MAX_SPEED * 0.55);

//     WHEEL_SPEEDS[0] += steer;
//     WHEEL_SPEEDS[3] += steer;
//     WHEEL_SPEEDS[2] += steer;
//     WHEEL_SPEEDS[1] += steer;

// #endif

//     for (int i = 0; i < 4; i++) {
//         WHEEL_SPEEDS[i] = constrain(WHEEL_SPEEDS[i], -WHEEL_STEPPER_MAX_SPEED, WHEEL_STEPPER_MAX_SPEED);
//         WHEEL_STEPPER_MOTORS[i].setSpeed(WHEEL_SPEEDS[i]);
//     }
// }

// void HOME_STEPPER(AccelStepper &Stepper_Motor, int End_Switch) {
//     int homing = 0;
//     Stepper_Motor.setCurrentPosition(0);
//     Stepper_Motor.setMaxSpeed(HOMING_SPEED);
//     Stepper_Motor.setAcceleration(HOMING_ACCELERATION);

//     Serial.print("Moving to the end switch...(");
//     Serial.print(End_Switch);
//     Serial.println(")");

//     while (!digitalRead(End_Switch)) {
//         Stepper_Motor.moveTo(homing);
//         homing--;
//         Stepper_Motor.run();
//     }

//     Stepper_Motor.setCurrentPosition(0);
//     Stepper_Motor.setMaxSpeed(HOMING_SPEED / 4);
//     Stepper_Motor.setAcceleration(HOMING_SPEED / 4);
//     homing = 1;
//     tone(BUZZER_PIN, 2000, 200);
//     Serial.println("Finalizing...");

//     while (digitalRead(End_Switch)) {
//         Stepper_Motor.moveTo(homing);
//         Stepper_Motor.run();
//         homing++;
//         delay(4);
//     }

//     Stepper_Motor.setCurrentPosition(0);
//     Stepper_Motor.setMaxSpeed(ROBOT_STEPPER_MAX_SPEED_DEFAULT);
//     Stepper_Motor.setAcceleration(ROBOT_STEPPER_MAX_ACCELERATION_DEFAULT);

//     Serial.print("Homing completed! (End switch pin number ");
//     Serial.print(End_Switch);
//     Serial.println(")");
//     Serial.println();
// }

// float Calculate_Battery_Voltage(int PIN_TO_READ) {
//     static float res = 1024.0;  // pow(2, 10);
//     static float R1 = 10.07;
//     static float R2 = 1.0;
//     static float Vcc = 3.3;
//     float Vout = Vcc * float(analogRead(PIN_TO_READ)) / res;
//     float Vs = Vout * (R1 + R2) / R2;

//     return Vs;
// }

// void GRIPPER_CONTROL(int state) {
//     switch (state) {
//         case 1:
//             digitalWriteFast(GRIPPER_OPEN, LOW);
//             digitalWriteFast(GRIPPER_CLOSE, LOW);
//             break;
//         case 2:
//             digitalWriteFast(GRIPPER_OPEN, HIGH);
//             digitalWriteFast(GRIPPER_CLOSE, LOW);
//             break;
//         case 3:
//             digitalWriteFast(GRIPPER_OPEN, LOW);
//             digitalWriteFast(GRIPPER_CLOSE, HIGH);
//             break;
//     }
// }

// void LED(char color) {
//     switch (color) {
//         case 'R':
//         case 1:
//             digitalWriteFast(RED_PIN, HIGH);
//             digitalWriteFast(GREEN_PIN, LOW);
//             digitalWriteFast(BLUE_PIN, LOW);
//             break;
//         case 'G':
//         case 2:
//             digitalWriteFast(RED_PIN, LOW);
//             digitalWriteFast(GREEN_PIN, HIGH);
//             digitalWriteFast(BLUE_PIN, LOW);
//             break;
//         case 'B':
//         case 3:
//             digitalWriteFast(RED_PIN, LOW);
//             digitalWriteFast(GREEN_PIN, LOW);
//             digitalWriteFast(BLUE_PIN, HIGH);
//             break;
//         case 'C':
//         case 4:
//             digitalWriteFast(RED_PIN, LOW);
//             digitalWriteFast(GREEN_PIN, HIGH);
//             digitalWriteFast(BLUE_PIN, HIGH);
//             break;
//         case 'M':
//         case 5:
//             digitalWriteFast(RED_PIN, HIGH);
//             digitalWriteFast(GREEN_PIN, LOW);
//             digitalWriteFast(BLUE_PIN, HIGH);
//             break;
//         case 'Y':
//         case 6:
//             digitalWriteFast(RED_PIN, HIGH);
//             digitalWriteFast(GREEN_PIN, HIGH);
//             digitalWriteFast(BLUE_PIN, LOW);
//             break;
//         case 'W':
//         case 7:
//             digitalWriteFast(RED_PIN, HIGH);
//             digitalWriteFast(GREEN_PIN, HIGH);
//             digitalWriteFast(BLUE_PIN, HIGH);
//             break;
//         case 'O':
//         case 0:
//             digitalWriteFast(RED_PIN, LOW);
//             digitalWriteFast(GREEN_PIN, LOW);
//             digitalWriteFast(BLUE_PIN, LOW);
//             break;
//         default:
//             //      digitalWriteFast(RED_PIN, LOW);
//             //      digitalWriteFast(GREEN_PIN, LOW);
//             //      digitalWriteFast(BLUE_PIN, LOW);
//             break;
//     }
// }
