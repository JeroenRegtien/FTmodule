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

// Simple program to determine three coordinates to program a movement sequence
// Disconnect the motors from the gears by removing the gear 10, the arms can now
// move freely. Prese E1 to ontin the coordinates.

#include <FTmodule.h>

#define ROTATE_PIN A0
#define SHOULDER_PIN A1
#define ELBOW_PIN A2

const char *programName = "RobotArm";
FTmodule shield(ADA_UNO, 1);
FTcontroller controller(NONE);

void setup() {
  Serial.begin(115200);
  controller.begin(programName);
  shield.begin();
}

void loop() {

  shield.getInputs();

  if (shield.getInput(ft_E1)) {
    Serial.print("{");
    Serial.print(shield.getDirectAnalog(ROTATE_PIN)); Serial.print(", ");
    Serial.print(shield.getDirectAnalog(SHOULDER_PIN)); Serial.print(", ");
    Serial.print(shield.getDirectAnalog(ELBOW_PIN)); Serial.print("} ");
    Serial.println();
  }
}

