/* 
 *  Prototype program for Controllino or ftDuino Module BL-1
 *  
 * currently uses ftDuino.
 * 
 * Jeroen Regtien 2024
 *
 * Interface:
 *    G: Ground
 *    O1: lopende band lamp 
 *    O2: lopende band motor links (uni directioneel)
 *    O3: laslamp
 *    O4: lopende band motor rechts (uni directioneel)
 *    E1: switch 1: fotocel start band
 *    E2: switch 2: schuiver links
 *    E3: switch 3: schuiver rechts
 *    E6: kolom onder
 *    E5: kolom boven
 *.   M3 (O5,O6) : kolom op/neer
 *    M4 (O7,O8) : transport motor
 *
 * functiebeschrijving
 * ===================
 * 1. Bij initialisatie gaat de schuifarm naar links en de kolomstaaf omhoog
 * 2. Wanneer de lichtcel aan het begin van de band wordt onderbroken gaat de band lopen
 * 3. De band loopt voor drie/vier seconden en stopt
 * 4. De kolomstaaf gaat naar beneden en stopt
 * 5. De lasbewerking vindt plaatse gedurende 3 seconden
 * 6. De kolomstaaf gaat omhoog
 * 7. De schuifarm gaat naar rechts
 * 8. De lopende band rechts loopt gedurende drie seconden
*/

// include the FT legacy controller classes
#include <FTmodule.h>

// define Program name
const char* programName = "ftDuino BL-1";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID_UNO, ADA_UNO, FTDUINO, etc},
// second is interface number for the type
FTmodule myFTduino(FT_DUINO, 1);

// argument is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

FTtimer transportTimer(3000);  // timer to set de dealy for the right belt after transport arm starts
FTtimer beltTimer(7000);       // timer for the duration of the belts
FTtimer firstTimer(2000);      // timer to check the initial position of the transort arm

// belt status parameters
bool beltStarted = false;
bool beltReady = false;
bool beltRightReady = false;

// arm status parameters
bool armLeft = false;
bool armRight = false;
bool armDown = false;
bool armRaised = false;
bool welderON = false;

void setup() {
  Serial.begin(115200);
  Serial.println(programName);

  controller.begin(programName);
  myFTduino.begin();
  myFTduino.setAllMotorsSTOP();
  beltTimer.reset();
  firstTimer.reset();
}

//---------------------------------------------------------------------------
// moves arm left until switch is closed
// returns true when condition is met, else false
bool moveArmLeft() {
  // Serial.println("move arm left");
  return (!myFTduino.getMotorUntil(ft_M4, ft_E2, ON, CW));
}

//---------------------------------------------------------------------------
// moves arm right until switch is closed
// returns true when condition is met, else false
bool moveArmRight() {
  // Serial.println("move arm right");
  return (!myFTduino.getMotorUntil(ft_M4, ft_E3, ON, CCW));
}

//---------------------------------------------------------------------------
// moves arm up until switch is closed
// returns true when condition is met, else false
bool moveArmUp() {

  return (!myFTduino.getMotorUntil(ft_M3, ft_E8, ON, CW));
}

//---------------------------------------------------------------------------
// moves arm down until switch is closed
// returns true when condition is met, else false
bool moveArmDown() {

  return (!myFTduino.getMotorUntil(ft_M3, ft_E5, ON, CCW));
}

//---------------------------------------------------------------------------
// Run belt for several seconds after parcel appears at gate
// Returns true when belt finally stopped, else false
bool startBelt(int E, int O) {  // run belt for several seconds

  // either wait for fotocell or start immediately (E==0)
  if (!myFTduino.getInput(E) || E == 0) {
    if (!beltStarted) {
      beltTimer.reset();
      beltStarted = true;
    }
  }

  if (beltStarted) {
    // stop the belt after beltDuration millis, continue running otherwise
    if (beltTimer.ready()) {
      myFTduino.setOutputOFF(O);
      beltStarted = false;
      return (true);
    } else {
      myFTduino.setOutputON(O);
      return (false);
    }
  } else {
    return (false);
  }
}

bool first = true;

//---------------------------------------------------------------------------
void loop() {


  // retrieve input from interfaces if any :: problem is that it alse sends output
  myFTduino.getInputs();

  // for the first pass ensure all the inital positions are taken by moving transport rigth.
  if (first) {
    if (firstTimer.ready()) {
      myFTduino.setMotorSTOP(ft_M4);
      first = false;
    } else {
      myFTduino.setMotorCCW(ft_M4);
    }

  } else {


    // move the arm to start position
    if (!armLeft) {
      armLeft = moveArmLeft();
      myFTduino.setOutputON(ft_O1);
    }

    // wait for a parcel to appear run the belt
    if (!beltReady && armLeft) {
      myFTduino.setOutputON(ft_O1);
      beltReady = startBelt(ft_E1, ft_O2);
    }

    if (beltReady) {

      // lower arm when left and parcel delivered
      if (armLeft && !armDown) {
        armDown = moveArmDown();
        myFTduino.setOutputOFF(ft_O1);
      }

      // initiate welder when arm down
      if (armDown && !welderON) {
        weldingLight();
        welderON = true;
      }

      // raise arm when done welding
      if (welderON && !armRaised) {
        armRaised = moveArmUp();
        transportTimer.reset();
        // myFTduino.setMotorSTOP(M3);
      }

      // move arm rigth when raised
      if (armRaised && !armRight) {
        armRight = moveArmRight();
        if (transportTimer.ready()) {
          beltRightReady = startBelt(0, ft_O4);
        }
      }

      // release suction
      if (armRaised && !beltRightReady) {
        // switch on belt right
        if (transportTimer.ready()) {
          beltRightReady = startBelt(0, ft_O4);
        }
      }

      // Reset all state variables
      if (beltRightReady) {
        armLeft = false;
        armRight = false;
        beltReady = false;
        armRaised = false;
        armDown = false;
        welderON = false;
        beltRightReady = false;
      }
    }
  }

  // update the display
  // myFTduino.ftUpdateDisplay();
}
//
//   This procedure simulates a welding action by combining random light pulses
//
void weldingLight() {
  int i, count;
  count = random(10, 60);
  for (i = 0; i < count; i++) {
    myFTduino.setOutputON(ft_O3);  // set the LED on
    delay(random(60));
    myFTduino.setOutputOFF(ft_O3);  // set the LED on
    delay(random(200));
  }
  delay(random(400, 1000));
}