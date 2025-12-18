/*******************************************************************************************
 * 
 *  Prototype program for Storage Module 1  VP-1UD
 *  
 *  Uses Arduino Uno / Didacta shield
 * 
 *  Jeroen Regtien 2024
 * 
 * Interface
 * =========
 *    G:  ground
 *    O1: hydraulic valve 1
 *    O2: hydraulic valve 2
 *    O3: hydraulic valve 3
 *    O5: compressor
 *    O7: motor // both parallel
 *    O7: lamp fotocel // both parallel
 *    O9: lamp red
 *    O10: lamp green
 *    E1: switch 1
 *    E2: switch 2
 *    E3: switch 3 
 *    E4: fotocel
 *
 * Description
 * ===========
 * 1. Initially the intallatiion is inactive. 
 * 2. When one of the three switches at the front is pressed, the band starts and the compressor is activated
 * 3. Pneumatic valve M1, M2, or M3 is opened and the from is pushed on the band 
 * 4. When the form passes the light cell, the band switches off after a short while
 * 5. As long as the band is rolling, no additional forms can be added to it.
*/

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "Opslag VP_1N";

unsigned long previousBeltMillis = 0;
unsigned long currentMillis;
const long delayConveyerBelt = 2000;
int countPass = 0;

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID, ADA},
// second is interface number for the type.
FTmodule interface(FT_NANO, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

// define timers
// FTtimer beltTimer(7000);
FTtimer valveTimer(2000);
FTtimer blockBeltTimer(8000);


void setup() {
  Serial.begin(9600);
  Serial.println(programName);

  controller.begin(programName);
  interface.begin();

  interface.setAllMotorsSTOP();
  // beltTimer.reset();
  previousBeltMillis = millis();
  blockBeltTimer.reset();
  valveTimer.reset();

  setWarningOFF();
}

bool triggeredPhotoCell = false;
bool beltLoaded = false;
bool beltRunning = false;
bool beltSwitchedOff = true;

//---------------------------------------------------------------------------
// switch to green light
bool setWarningOFF() {
  Serial.println("warning led off");
  interface.setOutputON(ft_O9);
  interface.setOutputOFF(ft_O10);
  return (true);
}

//---------------------------------------------------------------------------
// switch to red light
bool setWarningON() {
  Serial.println("warning led on");
  interface.setOutputOFF(ft_O9);
  interface.setOutputON(ft_O10);
  return (true);
}

//---------------------------------------------------------------------------
void loop() {

  // get the time in milliseconds
  currentMillis = millis();

  // retrieve input from interfaces if any :: problem is that it alse sends output
  interface.getInputs();

  // operate the belt
  if (beltRunning) {
    beltRunning = operateBelt();

    if (blockBeltTimer.ready()) {
      interface.setOutputOFF(ft_O7);
      interface.setOutputOFF(ft_O7);
      beltLoaded = false;
      beltRunning = false;
      beltSwitchedOff = true;
      Serial.println("stop load");
      setWarningOFF();
    }
  } else {
    // when switch is pressed, start compressor, open valve and start conveyer belt and light
    // but keep belt running until fotocel is triggered
    if (interface.getInput(ft_E1) || interface.getInput(ft_E2) || interface.getInput(ft_E3)) {

      if (interface.getInput(ft_E1)) {
        Serial.println("switch 1");
        interface.setOutputON(ft_O1);
      } else if (interface.getInput(ft_E2)) {
        Serial.println("switch 2");
        interface.setOutputON(ft_O2);
      } else if (interface.getInput(ft_E3)) {
        Serial.println("switch 3");
        interface.setOutputON(ft_O3);
      }

      interface.setOutputON(ft_O5);
      interface.setOutputON(ft_O7);
      interface.setOutputON(ft_O7);
      valveTimer.reset();
      blockBeltTimer.reset();
      setWarningON();
      beltLoaded = true;
    }

    else {
      if (beltLoaded) {
        // wait 2 seconds before turning everything off
        if (valveTimer.ready()) {
          interface.setOutputOFF(ft_O1);
          interface.setOutputOFF(ft_O2);
          interface.setOutputOFF(ft_O3);
          interface.setOutputOFF(ft_O5);
          valveTimer.reset();
          beltLoaded = false;
          beltRunning = true;
        }
      }
    }
  }

  // update the display
  // interface.ftUpdateDisplay();
}

//
// function retuns false when belt stop command is given to motor and lamp
//
bool operateBelt() {

  bool beltOn = true;

  countPass++;

  // for now assume fotocel gives digital s§ignal
  // if true stop de converyer belt and switch off the lamp
  if (!interface.getInput(ft_E4) && !triggeredPhotoCell) {
    Serial.println("end of belt reached");
    beltSwitchedOff = false;

    // start the timer
    if (!triggeredPhotoCell) {
      triggeredPhotoCell = true;
      // beltTimer.reset();
      previousBeltMillis = currentMillis;
      Serial.println("end of belt timer set");
    }
  } else {
    if (!triggeredPhotoCell && !beltSwitchedOff) {
      Serial.println("continue running belt");
      interface.setOutputON(ft_O7);// note both exit lamp and motor
      interface.setOutputON(ft_O7); 
    }
  }

  if (triggeredPhotoCell) {
    // if (beltTimer.ready()) {
    if (currentMillis - previousBeltMillis > delayConveyerBelt) {
      interface.setAllMotorsSTOP();
      triggeredPhotoCell = false;
      beltSwitchedOff = true;
      beltOn = false;
      Serial.println("belt switched off");
      setWarningOFF();
    }
  }

  return (beltOn);
}
