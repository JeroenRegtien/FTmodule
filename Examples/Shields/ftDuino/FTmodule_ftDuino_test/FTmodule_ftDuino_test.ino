/* 
 *  Prototype program for Controllino or ftDuino Module BL-1
 *  
 * currently uses ftDuino.
 * 
 * Jeroen Regtien 2024
 *
 * Interface:
 *    E1: input switch to start motor 2
 *    M1: (O1,O2) : motor to test variable speed
 *    M2: (O3,O4) : motor triggered by E1
 *    M3: (O5,O6) : lamp on O5
 *
 * functiebeschrijving
 * ===================
 * 1. switch 1 starts motor 2
 * 2. motor 1 goes through a variable speed cycle
 * 3. lamp 05 goes through a variable dim cycle
 * 4. all outputs are deactivated, and so forth
*/

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "ftDuino test";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID_UNO, ADA_UNO, FT_DUINO},
// second is interface number for the type.
FTmodule myFTduino(FT_DUINO, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);


void setup() {

  controller.begin(programName);
  myFTduino.begin();

  delay(2000);
  Serial.begin(115200);
  //Serial.println(programName);
}

bool first = true;

//---------------------------------------------------------------------------
void loop() {

  if (first) {
      Serial.println(programName);
      first = false;
  }
  // retrieve input from interfaces if any :: problem is that it alse sends output
  myFTduino.getInputs();
  myFTduino.printInputBuffer();

  if (myFTduino.getInput(ft_E1)) {
    myFTduino.setMotorCW(ft_M2);
  }

  myFTduino.setMotorCW(ft_M1);
  for (int i=0; i<8; i++) {
    myFTduino.setMotorSpeed(ft_M1, 8*i);
    delay(2000);
  }
  for (int i=0; i<8; i++) {
    myFTduino.setMotorSpeed(ft_M1, 64 - 8*i);
    delay(2000);
  }

  myFTduino.setOutputON(ft_O5);
    for (int i=0; i<4; i++) {
    myFTduino.setMotorSpeed(ft_M3, 16*i);
    delay(2000);
  }

  myFTduino.setAllMotorsSTOP();
  // update the display
  myFTduino.ftUpdateDisplay();
}
