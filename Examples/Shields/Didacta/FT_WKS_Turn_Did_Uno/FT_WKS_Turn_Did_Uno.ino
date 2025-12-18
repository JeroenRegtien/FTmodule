
/* 
 *  Prototype program for Controllino Storage Module 1
 *  
 * uses two serial interfaces
 * 
 * Interface 1:
 *    O1: lamp
 *    O2: lopende band motor
 *.   O3: rode lamp
 *.   O4: groene lamp
 *    M3: keer motor
 *    M4: grijper motor
 *    E1: switch 1: fotocel start band
 *    E2: switch 2: positie grijper entree
 *    E3: switch 3: positie grijper uitgang
 *.   E4: switch 4: grijper open
 *
 * functiebeschrijving
 * ===================
 * 1. Bij initialisatie gaat de grijper naar links met open grijper (rust stand)
 * 2. Wanneer de lichtcel aan het begin van de band wordt onderbroken gaat de band lopen, 
 *    maar alleen als de grijper in ruststand is
 * 3. De band loopt voor vier seconden en stopt
 * 4. De grijper bevindt zich in ruststand links op de band met open grijper, de grijper gaat dicht
 * 5. De griper roteert tot de uitgangsstand rechts boven de glijbaan
 * 6. De grijper opent
 * 7. De grijper gaat naar de ruststand 
 */

unsigned long previousBeltMillis = 0;
unsigned long previousGripperMillis = 0;
unsigned long currentMillis;
unsigned long beltStartMillis;
unsigned long gripperStartMillis;
const long beltDurationMillis = 4000;
const long gripperDurationMillis = 1400;
int countPass = 0;

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "Cont. Keren 1";

// create interface objects. first argument = {PAR, SER, ROBO, DID, ADA},
// second is interface number for the type.
FTmodule i1(DID_UNO, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(NONE);

// belt status parameters
bool beltLoaded = false;
bool beltRunning = false;
bool beltSwitchedOff = true;
bool beltStarted = false;
bool beltReady = false;

// gripper status parameters
bool gripperLeft = false;
bool gripperRight = false;
bool gripperOpen = false;
bool gripperClosed = false;
bool gripperFirst = true;


void setup() {
  Serial.begin(9600);
  Serial.println(programName);

  // prepare for serial port to be used in library
  controller.addInterface(i1);
  controller.begin(programName);
  i1.begin();

  i1.setAllMotorsSTOP();
  i1.setOutputON(ft_O4);
  currentMillis = millis();
  previousBeltMillis = currentMillis;
  previousGripperMillis = currentMillis;
}

//---------------------------------------------------------------------------
// moves gripper to counterclockwise until switch is closed
// returns true when condition is met, else false
bool moveGripperLeft() {
  return (!i1.getMotorUntil(ft_M3, ft_E2, ON, CW));
}

//---------------------------------------------------------------------------
// moves gripper to clockwise until switch is closed
// returns true when condition is met, else false
bool moveGripperRight() {
  return (!i1.getMotorUntil(ft_M3, ft_E3, ON, CCW));
}

//---------------------------------------------------------------------------
// opens gripper
// returns true when condition is met, else false
bool openGripper() {
  return (!i1.getMotorUntil(ft_M4, ft_E4, ON, CCW));
}

//---------------------------------------------------------------------------
// closes gripper
bool closeGripper() {

  if (gripperFirst) {
    gripperStartMillis = currentMillis;
    gripperFirst = false;
  }
  if ((currentMillis - gripperStartMillis) > gripperDurationMillis) {
    i1.setMotorSTOP(ft_M4);
    return (true);
  } else {
    i1.setMotorCW(ft_M4);
    return (false);
  }
}

//---------------------------------------------------------------------------
// Run belt for several seconds after parcel appears at gate
// Returns true when belt finally stopped, else false
bool startBelt() {  // run belt for several seconds

  if (!i1.getInput(ft_E1)) {
    if (!beltStarted) {
      beltStartMillis = currentMillis;
      beltStarted = true;
    }
  } 

  if (beltStarted) {
    // stop the belt after beltDuration millis, continue running otherwise
    if ((currentMillis - beltStartMillis) > beltDurationMillis) {
      i1.setOutputOFF(ft_O2);
      beltStarted = false;
      return (true);
    } else {
      i1.setOutputON(ft_O2);
      i1.setOutputON(ft_O3);
      i1.setOutputOFF(ft_O4);
      return (false);
    }

  }
  else {
    return(false);
  }
}

//---------------------------------------------------------------------------
void loop() {

  // get the time in milliseconds
  currentMillis = millis();

  // retrieve input from interfaces if any
  i1.getInputs();

  // move the gripper to start position
  if (!gripperLeft) {
    gripperLeft = moveGripperLeft();
    i1.setOutputON(ft_O1);  // needs to be early otherwise triggers band
    if (gripperLeft) {
      i1.setOutputON(ft_O4);
      i1.setOutputOFF(ft_O3);
    }
  } 

  // wait for a parcel to appear; run the belt; switch on red light, switched off green light
  if (!beltReady && gripperLeft) {
    beltReady = startBelt();
  } else {
  }

  if (beltReady) {

    // close gripper when left and parcel delivered
    if (gripperLeft && !gripperClosed) {
      gripperClosed = closeGripper();
      i1.setOutputOFF(ft_O1);
    }

    // rotate gripper to end position
    if (gripperClosed && !gripperRight) {
      gripperRight = moveGripperRight();
    }

    // open gripper 
    if (gripperRight) {
      gripperOpen = openGripper();
    }

    // Reset all state variables
   if (gripperOpen) {
      gripperLeft = false;
      gripperRight = false;
      beltReady = false;
      gripperClosed = false;
      gripperFirst = true;
      gripperOpen = false;
    }
  }

  // update the display
  i1.ftUpdateDisplay();
}
