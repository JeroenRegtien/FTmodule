/* 
 *  Prototype program for Robot Arm from book 
 *  'fischertechnik roboter mit Arduino'
 *  Dirk Fox, Thomas Puttman, ISBM 978-86490-426-4
 *  
 *  User Adafruit Motorshield V2 in combination with FTmodule
 * Copyright (c) 2025 Jeroen Regtien
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **********************************************************************************************/

/*
  In the Arduino IDE serial monitor a number of commands can be given 
  to test individual robot arm movements.

  's' : all motors stop
  'd' : upper arm (elbow) down
  'u' : upper arm (elbow) up
  'f' : lower arm (shoulder) forward
  'b' : lower arm (shoulder)backward
  'l' : rotate left
  'r' : rotate right
  'c' : close gripper
  'o' : open gripper

*/
#include <FTmodule.h>

#define ROTATE ft_M1
#define ROTATE_PIN A0
#define SHOULDER ft_M2
#define SHOULDER_PIN A1
#define ELBOW ft_M3
#define ELBOW_PIN A2
#define GRIPPER ft_M4

const char *programName = "RobotArm";
FTmodule shield(ADA_UNO, 1);
FTcontroller controller(NONE);

bool armForward(int steps);
bool armDown(int steps);
bool rotate(int steps);
bool gripperOpen();
bool gripperClose();

// min and max speeds for the motors
const int min_v_forward[4] = { 255, 100, 100, 100 };
const int max_v_forward[4] = { 255, 200, 200, 255 };
const int min_v_backward[4] = { 255, 100, 100, 100 };
const int max_v_backward[4] = { 255, 200, 200, 255 };

// data structure to store sequence of events for learning mode
struct Position {
  int angle;
  int shoulder;
  int elbow;
  int gripper;
};

// array of demonstation positions
const int demoIndex = 16;
Position demo[demoIndex] = { 
  { 500, 535, 750, 0 }, 
  { 500, 535, 400, 1 }, 
  { 500, 200, 400, 1 },
  { 300, 200, 400, 1 },
  { 300, 535, 400, 0 }, 
  { 300, 535, 750, 0 },
  { 500, 535, 750, 0 },
  { 500, 200, 200, 0 },
  { 300, 535, 750, 0 },
  { 300, 535, 400, 1 }, 
  { 300, 200, 400, 1 },
  { 500, 200, 400, 1 },
  { 500, 535, 400, 0 }, 
  { 500, 535, 750, 0 }, 
  { 300, 535, 750, 0 },
  { 300, 250, 200, 0 } };

void setup() {
  Serial.begin(115200);
  controller.begin(programName);
  shield.begin();
  resetAllStations();
}

String command, numberChar, choice;
int number;

void loop() {

  shield.getInputs();
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    Serial.println(command);

    if (choice == "s") { Serial.println(F("stop")); shield.setAllMotorsSTOP(); }

    if (choice == "D") { Serial.println(F("down")); move(ELBOW, ELBOW_PIN, 300); }
    if (choice == "d") { Serial.println(F("down step")); moveRelative(ELBOW, ELBOW_PIN, 25); }
    if (choice == "U") { Serial.println(("up")); move(ELBOW, ELBOW_PIN, 100); }
    if (choice == "u") { Serial.println(("up step")); moveRelative(ELBOW, ELBOW_PIN, -25); }

    if (choice == "L") { Serial.println(F("left")); move(ROTATE, ROTATE_PIN, 300); }
    if (choice == "R") { Serial.println(F("right")); move(ROTATE, ROTATE_PIN, 400); }
    if (choice == "l") { Serial.println(F("left step")); moveRelative(ROTATE, ROTATE_PIN, -25); }
    if (choice == "r") { Serial.println(F("right step")); moveRelative(ROTATE, ROTATE_PIN, 25); }

    if (choice == "F") { Serial.println(F("forward")); move(SHOULDER, SHOULDER_PIN, 500); }
    if (choice == "f") { Serial.println(F("forward step")); moveRelative(SHOULDER, SHOULDER_PIN, 25); }
    if (choice == "B") { Serial.println(F("backward")); move(SHOULDER, SHOULDER_PIN, 150); }
    if (choice == "b") { Serial.println(F("backward step")); moveRelative(SHOULDER, SHOULDER_PIN, -25); }

    if (choice == "c") { Serial.println(F("close")); gripperClose(); }
    if (choice == "o") { Serial.println(F("open")); gripperOpen(); }
    if (choice == "m") { Serial.println(F("demo")); executeDemo(); }
    if (choice == "i") { resetAllStations(); }
  }
}

//----------------------------------------------------------------------------
// execute the demo
void executeDemo() {
int r, u, l, g;

  resetAllStations();

  for (int i=0; i<demoIndex; i++) {
    r = demo[i].angle;
    u = demo[i].shoulder;
    l = demo[i].elbow;
    g = demo[i].gripper;
    Serial.print("Move to point: ");
    Serial.print(r);
    Serial.print(",");
    Serial.print(u);
    Serial.print(",");
    Serial.println(l);

    move(SHOULDER, SHOULDER_PIN, u);
    move(ELBOW, ELBOW_PIN, l);
    move(ROTATE, ROTATE_PIN, r);
    if (g==1) {
       gripperClose();
    } else {
      gripperOpen();
    }
  }
}

//----------------------------------------------------------------------------
// reset rotation
bool resetRotation() {
  move(ROTATE, ROTATE_PIN, 200);
};

//----------------------------------------------------------------------------
// reset elbow (lower) arm
bool resetArmElbow() {
  move(ELBOW, ELBOW_PIN, 200);
};

//----------------------------------------------------------------------------
// reset shoulder (upper) arm
bool resetArmShoulder() {
  move(SHOULDER, SHOULDER_PIN, 100);
};

//----------------------------------------------------------------------------
// reset all stations to avoid interference or clashes
bool resetAllStations() {

  resetArmElbow();
  resetArmShoulder();
  resetRotation();
  gripperOpen();
  return (true);
}

//----------------------------------------------------------------------------
// open the gripper
bool gripperOpen() {
  shield.setMotorCCW(GRIPPER);
  shield.setMotorSpeed(GRIPPER, max_v_backward[GRIPPER-1]);
  delay(900);
  shield.setMotorSpeed(GRIPPER, min_v_backward[GRIPPER-1]);
  delay(1800);
  shield.setMotorSTOP(GRIPPER);
  return(true);
}

//----------------------------------------------------------------------------
// close the gripper 
bool gripperClose() {
  shield.setMotorCW(GRIPPER);
  shield.setMotorSpeed(GRIPPER, max_v_forward[GRIPPER-1]);
  delay(900);
  shield.setMotorSpeed(GRIPPER, min_v_forward[GRIPPER-1]);
  delay(2000);
  shield.setMotorSTOP(GRIPPER);
  return (false);
}

//----------------------------------------------------------------------------
// make an absolute movement 
void move(int motor, int motorPin, int target) {
    
  int currentPos, speed;
  int tolerance = 3;
  bool go_on = true;

  while ( go_on ) {
    
    currentPos = shield.getDirectAnalog(motorPin);
    // Serial.println(currentPos);
    if (currentPos > target + tolerance) {
       speed = min (max_v_backward[motor-1], target-currentPos);
       speed = max (speed, min_v_backward[motor-1]);
       shield.setMotorSpeed(motor, speed);
       shield.setMotorCW(motor);
     }
     else if (currentPos < target - tolerance) {
       speed = min (max_v_forward[motor-1], target-currentPos);
       speed = max (speed, min_v_forward[motor-1]);
       shield.setMotorSpeed(motor, speed);
       shield.setMotorCCW(motor);
     }
     else {
       shield.setMotorSTOP(motor);
       go_on = false;
     }
  }
  Serial.print(F("New position: "));
  Serial.println(target);
}

//----------------------------------------------------------------------------
// make a relative movement 
void moveRelative(int motor, int motorPin, int delta) {
   
  int target = delta + shield.getDirectAnalog(motorPin);
  move(motor, motorPin, target);
 }