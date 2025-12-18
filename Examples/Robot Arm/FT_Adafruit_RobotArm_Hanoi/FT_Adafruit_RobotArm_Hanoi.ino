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

// settings for motor speed
const int min_v_forward[4] = { 180, 100, 100, 100 };
const int max_v_forward[4] = { 255, 150, 150, 255 };
const int min_v_backward[4] = { 180, 90, 80, 100 };
const int max_v_backward[4] = { 255, 150, 150, 255 };

const int minPosition[4] = { 100, 200, 50, 0 };
const int maxPosition[4] = { 800, 950, 700, 200 };

// the arm positions for the three locations and situations
int positions[3][3][3] =    // THESE SHOULD BE RECALIBRATED!
  { { {300, 410, 540}, {300, 526, 540}, {300, 601, 540} },
    { {375, 446, 604}, {375, 557, 604}, {375, 677, 604} },
    { {450, 414, 671}, {450, 495, 671}, {450, 615, 671} } };

// simplified method
int positionRotation[3] = { 220, 290, 360};
int positionShoulder[3] = { 923, 725, 497};
int positionElbow[3] = { 399, 421, 465};

int numDiscs = 2;
int stack[3] = {numDiscs, 0, 0};
String command, choice;

void setup() {
  Serial.begin(9600);
  controller.begin(programName);
  shield.begin();
  resetAllStations();
}

void loop() {

  shield.getInputs();

  if (shield.getInput(ft_E1)) {
    towerHanoi(numDiscs, 0, 2, 1);    // array index starts ate zero  
  }

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);

    if (choice == "s") { Serial.println("stop"); shield.setAllMotorsSTOP(); }

    if (choice == "D") { Serial.println("down"); move(ELBOW, ELBOW_PIN, 300); }
    if (choice == "d") { Serial.println("down step"); moveRelative(ELBOW, ELBOW_PIN, 25); }
    if (choice == "U") { Serial.println("up"); move(ELBOW, ELBOW_PIN, 100); }
    if (choice == "u") { Serial.println("up sep"); moveRelative(ELBOW, ELBOW_PIN, -25); }

    if (choice == "L") { Serial.println("left"); move(ROTATE, ROTATE_PIN, 300); }
    if (choice == "R") { Serial.println("right"); move(ROTATE, ROTATE_PIN, 400); }
    if (choice == "l") { Serial.println("left step"); moveRelative(ROTATE, ROTATE_PIN, -25); }
    if (choice == "r") { Serial.println("right step"); moveRelative(ROTATE, ROTATE_PIN, 25); }

    if (choice == "F") { Serial.println("forward"); move(SHOULDER, SHOULDER_PIN, 500); }
    if (choice == "f") { Serial.println("forward step"); moveRelative(SHOULDER, SHOULDER_PIN, 25); }
    if (choice == "B") { Serial.println("backward"); move(SHOULDER, SHOULDER_PIN, 150); }
    if (choice == "b") { Serial.println("backward step"); moveRelative(SHOULDER, SHOULDER_PIN, -25); }

    if (choice == "c") { Serial.println("close"); gripperClose(); }
    if (choice == "o") { Serial.println("open"); gripperOpen(); }

    if (choice == "i") { resetAllStations(); }
  }
}

//------------------------------------------------------------------------
// Recursive Tower of Hanoi routine
void towerHanoi (int n, int source, int destination, int store) {

  //interface.getInputs();  // alway need getInputs, otherwise timeout
  //interface.getAnalogInputs();
  // interface.printInputBuffer();

  if (n==1) { 
    moveDisc(source, destination);
    return;
  }

  // move n-1 disc from source to store
  towerHanoi(n-1, source, store, destination);
  moveDisc(source, destination);
  towerHanoi(n-1, store, destination, source);
}

//------------------------------------------------------------------------
// Move disk from source to destination
void moveDisc(int source, int destination) {
  move(ROTATE, ROTATE_PIN, positionRotation[source]);
  move(ELBOW, ELBOW_PIN, positionElbow[source]); 
  move(SHOULDER, SHOULDER_PIN, positionShoulder[source]); 
  gripperClose();
  move(SHOULDER, SHOULDER_PIN, 500);
  stack[source-1]--;

  move(ROTATE, ROTATE_PIN, positionRotation[destination]);
  move(ELBOW, ELBOW_PIN, positionElbow[destination]); 
  move(SHOULDER, SHOULDER_PIN, positionShoulder[destination]); 
  gripperOpen();
  move(SHOULDER, SHOULDER_PIN, 500);
  stack[destination-1]++;
}


bool gripperOpen() {

  shield.setMotorCCW(GRIPPER);
  shield.setMotorSpeed(GRIPPER, max_v_backward[GRIPPER-1]);
  delay(900);
  shield.setMotorSpeed(GRIPPER, min_v_backward[GRIPPER-1]);
  delay(1800);
  shield.setMotorSTOP(GRIPPER);

  return (true);
}

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
  Serial.print(F("New "));
  Serial.print(motor);
  Serial.print(" position: ");
  Serial.println(target);
}

//----------------------------------------------------------------------------
// make a relative move
void moveRelative(int motor, int motorPin, int delta) {
   
  int target = delta + shield.getDirectAnalog(motorPin);
  move(motor, motorPin, target);
 }

//----------------------------------------------------------------------------
// reset all stations to avoid interference or clashes
bool resetAllStations() {

  resetRotation();
  resetArmForward();
  resetArmUpDown();
  gripperOpen();
  return (true);
}

//----------------------------------------------------------------------------
// reset rotation
bool resetRotation() {
  move(ROTATE, ROTATE_PIN, (minPosition[ROTATE-1]+maxPosition[ROTATE-1])/2);
};

//----------------------------------------------------------------------------
// reset elbow (lower) arm
bool resetArmUpDown() {
  move(ELBOW, ELBOW_PIN, (minPosition[ELBOW-1]+maxPosition[ELBOW-1])/2-200);
};

//----------------------------------------------------------------------------
// reset shoulder (upper) arm
bool resetArmForward() {
  move(SHOULDER, SHOULDER_PIN, (minPosition[SHOULDER-1]+maxPosition[SHOULDER-1])/2);
};