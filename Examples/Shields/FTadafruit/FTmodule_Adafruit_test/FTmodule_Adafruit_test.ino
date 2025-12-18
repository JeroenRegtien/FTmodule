/* 
 *  Simple test program for Adafruit Motorshield V2
 *  
 * Jeroen Regtien 2025
 * 
 * There are four motos connected and one switch.
 * if switch ft_E1 is pressed, a cycle starts where one by 
 * one the motors will increase and decrease speed
 *
*/

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "TST-ADA";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID_UNO, ADA_UNO, etc},
// second is interface number for the type.
FTmodule interface(ADA_UNO, 1);

// second is the display {D1604, D1602, D2004}
FTcontroller controller(D2004);

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  interface.begin();

  // Set the speed to start, from 0 (off) to 255 (max speed)
 interface.setMotorSpeed(ft_M1, 0);
 interface.setMotorSpeed(ft_M2, 0);
 interface.setMotorSpeed(ft_M3, 0);
 interface.setMotorSpeed(ft_M4, 0);
}

void loop() {
  uint8_t i, m;
  int M;

  interface.getInputs();
  interface.getAnalogInputs();
  interface.printInputBuffer();

  if (interface.getInput(ft_E1)) {
    Serial.println("do in loop");
    for (m = 0; m < 4; m++) {
      M = m + 1;
      interface.setMotorCW(M);
      for (i = 0; i < 255; i++) {
        interface.setMotorSpeed(M, i);
        delay(10);
      }
      for (i = 255; i != 0; i--) {
        interface.setMotorSpeed(M, i);
        delay(10);
      }

      interface.setMotorCCW(M);
      for (i = 0; i < 255; i++) {
        interface.setMotorSpeed(M, i);
        delay(10);
      }
      for (i = 255; i != 0; i--) {
        interface.setMotorSpeed(M, i);
        delay(10);
      }
      interface.setMotorSTOP(M);
    }

  }
  
}