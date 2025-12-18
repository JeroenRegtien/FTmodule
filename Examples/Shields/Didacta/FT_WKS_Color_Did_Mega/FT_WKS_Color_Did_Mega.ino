/*******************************************************************************************
 * 
 *  Prototype program for Color Identification Module 1  VK-1MD
 *  
 *  Uses Arduino MEGA / Didacta shield
 * 
 * Interface
 * =========
 *    O1: lamp
 *    O2: motor band
 *    O3: compressor
 *    O4: hydraulic valve 2 - suction
 *    O5: hydraulic valve 3 - op/down
 *    O6: power up color sensor
 *    M4 (O7,O8): motor turntable
 *    O9: lamp red
 *    O10: lamp white
 *    O11: lamp blue
 *    E1: foto transistor
 *    E2: switch home position
 *    E3: switch rotation
 *    E4: signal color detector
 *    
 *
 * Description
 * ===========
 * 1. Initially, the installation is inactive, the arm turns to home position if not already there, the entrance lamp/LED is on
 * 2. When a form arrives and the photoelectric sensor is triggered the band starts for three seconds
 * 3. The compressor is turned on as well as the valve to lower the arm
 * 4. When the arm is down the suction valve is activated
 * 5. The arm is raised and rotated to the coloursensor location, the arm is lowered
 * 6. The coloursensor is activated and colour is measured
 * 7. Once a colour is derived (red, blue or white) the corresponding lamp/led is switched on
 * 8. The arm rotates further clockwise, at 180 degrees it is lowered and drops the form.
 * 9. The installation resets itself and awaits a next form to appear.

*/

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "VK-1MD Kleur";

// create interface objects. first argument = {PAR, SER, ROBO, DID, ADA},
// second is interface number for the type.
FTmodule interface(DID_MEGA, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

FTtimer beltTimer(6000);
FTtimer armTimer(2000);
FTtimer suctionTimer(2000);
FTtimer colorTimer(5000);

// belt status parameters
bool beltSwitchedOff = true;
bool beltStarted = false;
bool beltReady = false;

// arm status parameters
bool armLeft = false;
bool armMiddle = false;
bool armRight = false;
bool armDown1 = false;
bool armUp1 = false;
bool armDown2 = false;
bool armUp2 = false;
bool armDown3 = false;
bool armUp3 = false;

// pneumatic status paramerters
bool suctionON = false;
bool suctionOFF = false;
bool compON = false;

bool colorID = false;
bool colorON = false;

bool start = true;

void setup() {
  Serial.begin(9600);
  Serial.println(programName);

  controller.begin(programName);
  interface.begin();

  beltTimer.reset();
  interface.setOutputON(ft_O1);
}

//---------------------------------------------------------------------------
// moves arm left until switch is closed
// returns true when condition is met, else false
bool moveArmLeft() {
  return (!interface.setMotorUntil(ft_M4, ft_E2, ON, CW));
}

//---------------------------------------------------------------------------
// moves arm right until switch is closed
// returns true when condition is met, else false
bool moveArmRight(int count) {
  return (!interface.setMotorUntilCount(ft_M4, ft_E3, ON, CCW, count));
}

//---------------------------------------------------------------------------
// lowers arm
// returns true when condition is met, else false
bool moveArmDown() {
  interface.setOutputON(ft_O5);
  if (armTimer.ready()) {
    return (true);
  } else {
    return (false);
  }
}

//---------------------------------------------------------------------------
// raised arm
// returns true when condition is met, else false
bool moveArmUp() {
  interface.setOutputOFF(ft_O5);
  return (true);
}

//---------------------------------------------------------------------------
// initiate suction by switching on compressor and setting valve on
// returns true when condition is met, else false
bool setSuctionON() {
  interface.setOutputON(ft_O4);
  if (suctionTimer.ready()) {
    return (true);
  } else {
    return (false);
  }
}

//---------------------------------------------------------------------------
// release suction
// returns true when condition is met, else false
bool setSuctionOFF() {
  interface.setOutputOFF(ft_O4);
  return (true);
}

//---------------------------------------------------------------------------
// switch on compressor
// returns true when condition is met, else false
bool setCompressorON() {
  interface.setOutputON(ft_O3);
  return (true);
}

//---------------------------------------------------------------------------
// switch off compressor
// returns true when condition is met, else false
bool setCompressorOFF() {
  interface.setOutputOFF(ft_O3);
  return (false);
}

//---------------------------------------------------------------------------
// Run belt for several seconds after parcel appears at gate
// Returns true when belt finally stopped, else false
bool startBelt(int E, int Out) {  // run belt for several seconds

  // either wait for fotocell or start immediatel (E==0)
  if (!interface.getInput(E) || E == 0) {
    if (!beltStarted) {
      beltTimer.reset();
      beltStarted = true;
    }
  }

  if (beltStarted) {
    // stop the belt after beltDuration millis, continue running otherwise
    if (beltTimer.ready()) {
      interface.setOutputOFF(Out);
      beltStarted = false;
      return (true);
    } else {
      interface.setOutputON(Out);
      return (false);
    }
  } else {
    return (false);
  }
}

bool determineColor() {
  // test values
  // no form / open: 918-941
  // black: 919
  // blue: 916-922
  // red: 899-905
  // yellow: 887-893
  // white: 879-883

  int colorCode = interface.getDirectAnalog(ft_E4);
  Serial.println(String(colorCode));
  if ((colorCode > 915) && (colorCode < 919))  // BLUE
  {
    interface.setOutputON(ft_O9);
  } else {
    interface.setOutputOFF(ft_O9);
  }

  if (colorCode > 898 && colorCode < 906)  // RED
  {
    interface.setOutputON(ft_O10);
  } else {
    interface.setOutputOFF(ft_O10);
  }

  if (colorCode > 875 && colorCode < 895)  // WHITE
  {
    interface.setOutputON(ft_O11);
  } else {
    interface.setOutputOFF(ft_O11);
  }

  // take some time to get a reading.
  if (colorTimer.ready()) {
    return (true);
  } else {
    return (false);
  }
}

//---------------------------------------------------------------------------
void loop() {

  interface.getInputs();
  // interface.printInputBuffer();

  if (interface.getInput(ft_E2) && !beltReady) {
    interface.setOutputON(ft_O1);
    armLeft = true;
  }

  // move the arm to start position
  if (!armLeft) {
    armLeft = moveArmLeft();
    interface.setOutputON(ft_O1);
  }

  // wait for a parcel to appear run the belt
  if (!beltReady && armLeft) {
    beltReady = startBelt(ft_E1, ft_O2);
    armTimer.reset();
  }

  if (beltReady) {

    // lower arm when left and parcel delivered
    if (armLeft && !armDown1) {
      compON = setCompressorON();
      armDown1 = moveArmDown();
      interface.setOutputOFF(ft_O1);
      suctionTimer.reset();
    }

    // initiate suction when arm down
    if (armDown1 && !suctionON) {
      suctionON = setSuctionON();
      interface.setOutputOFF(ft_O1);
    }

    // raise arm when suction in place
    if (suctionON && !armUp1) {
      armUp1 = moveArmUp();
    }

    // move arm rigth when raised
    if (armUp1 && !armMiddle) {
      armMiddle = moveArmRight(300);
    }

    // move arm down and switch on color detector
    if (armMiddle && !armDown2) {
      armDown2 = moveArmDown();
      interface.setOutputON(ft_O6);
      colorTimer.reset();
    }

    if (armDown2 && !colorID) {
      colorID = determineColor();
    }

    // move arm up and switch off color detector
    if (colorID && !armUp2) {
      armUp2 = moveArmUp();
      interface.setOutputOFF(ft_O6);
    }

    // move arm right
    if (armUp2 && !armRight) {
      armRight = moveArmRight(300);
      armTimer.reset();
    }

    // move arm down & release
    if (armRight && !armDown3) {
      armDown3 = moveArmDown();
      suctionOFF = setSuctionOFF();
    }

    // release form
    if (armDown3 && !armUp3) {
      armUp3 = moveArmUp();
      setCompressorOFF();
    }

    // Reset all state variables
    if (armUp3) {  // armRight
      armLeft = false;
      armMiddle = false;
      armRight = false;
      beltReady = false;
      armUp1 = false;
      armDown1 = false;
      armUp2 = false;
      armDown2 = false;
      armUp3 = false;
      armDown3 = false;
      suctionON = false;
      suctionOFF = false;
      colorID = false;

      // switch off color LEDs
      interface.setOutputOFF(ft_O9);
      interface.setOutputOFF(ft_O10);
      interface.setOutputOFF(ft_O11);
    }
  }
  // update the display
  interface.ftUpdateDisplay();
}

