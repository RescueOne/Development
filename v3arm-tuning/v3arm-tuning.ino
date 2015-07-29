#include <avr/EEPROM.h>
#include <phys253.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

//To determine where on course the robot is
volatile unsigned int NUM = 0;

/*
=============
== CLASSES ==
=============
*/
class MenuItem
{
  public:
  String    Name;
  uint16_t  Value;
  uint16_t* EEPROMAddress;
  static uint16_t MenuItemCount;
  MenuItem(String name)
  {
    MenuItemCount++;
    EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
    Name      = name;
    Value         = eeprom_read_word(EEPROMAddress);
  }
  void Save()
  {
    eeprom_write_word(EEPROMAddress, Value);
  }
};

// Initializing the menu
uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGainTape = MenuItem("P-gain Tape");
MenuItem DerivativeGain   = MenuItem("D-gain Tape");
MenuItem IntegralGain     = MenuItem("I-gain Tape");
MenuItem ThresholdTape    = MenuItem("Thresh Tape");
MenuItem menuItems[]      = {Speed, ProportionalGainTape, DerivativeGain, IntegralGain, ThresholdTape};

/*
===========
== SETUP ==
===========
*/

void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
}

/*
==================
== TINAH INPUTS ==
==================
*/

//Motor
const int MOTOR_LEFT = 3; //PWM output for left motor
const int MOTOR_RIGHT = 2; //PWM output for right motor
const int MOTOR_CRANE_HEIGHT = 1; //Motor for arm height
const int MOTOR_CRANE_ANGLE = 0; //Motor for arm angle

//Analog
const int QRD_LEFT = 1; //Left QRD for tape following
const int QRD_RIGHT = 0; //Right QRD for tape following
const int QRD_PET_BACK = 2; //QRD for locating pets back
const int QRD_PET_FRONT = 3; //QRD for locating pets front
const int POTENTIOMETER_CRANE_HEIGHT = 5; //Rotary potentiometer for crane arm
const int POTENTIOMETER_CRANE_ANGLE = 4; //Rotary potentiometer for crane arm

//Servo
const int SERVO_PLATE = 0; //Servo to drop pet

//Digital
const int SWITCH_PLATE = 3; //Switch to see if pet is on plate
const int ROT_LEFT = 0; //Rotary encoder for left wheel
const int ROT_RIGHT = 1; //Rotary encoder for right wheel

/*
===========================
== ARM CONTROL CONSTANTS ==
===========================
*/

// Angle -> horizontal movement of the arm
// Height -> vertical movement of the arm

// Speed
const int SPEED_HEIGHT = 120;
const int SPEED_ANGLE = 80;

// PID Constants
const int P_HEIGHT = 20;
const int P_ANGLE = 10;
const int I_HEIGHT = 24;
const int I_ANGLE = 0;
const int I_MAX_HEIGHT = 150;
const int I_MAX_ANGLE = 150;
const int D_HEIGHT = 0;
const int D_ANGLE = 4;

// Positions
const int ARM_UP = 900;
const int ARM_HOR = 830;
const int ARM_DOWN = 700;
const int ARM_PICKUP = 600;
const int ARM_LEFT = 250;
const int ARM_CENTRE = 500;
const int ARM_RIGHT = 700;
const int SHIFT = 30; // The amount the arm shifts on each attempt

// Range of where the arm will be in an "error-free" zero
const int DEADBAND_HEIGHT = 15;
const int DEADBAND_ANGLE = 10;

// Other
const int MAX_TIME_LONG = 2000; //Max time the arm can move down for pickup (low pet)
const int MAX_TIME_SHORT = 1000; //Max time the arm can move down for pickup (high pet)
const int MAX_ANALOG = 1023; // for converting arduino resolution to speed
const int PET_QRD_THRESHOLD = 400; // when the arm will stop to pick up pets

//For reference
const int HEIGHT = 1;
const int ANGLE = 2;

/*
=================
== HOME SCREEN ==
=================
*/

void loop()
{
  setServo(SERVO_PLATE, 0);
  // ArmPID(HEIGHT, ARM_UP);

  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Stop: Arm Tuning");
  delay(100);

  // opens the menu
  if (startbutton()) {
    delay(100);
    if (startbutton()) { Menu(); }
  }

  // runs the control loop
  if (stopbutton()) {
    delay(100);
    if (stopbutton()) { mainStart(); }
  }
}

/*
==================
== CONTROL LOOP ==
==================
*/

/*
Main loop that controls the robot and the specific code required to get each pet.
NUM represents the pet/location
NUM == 0; start of course
NUM == 1; past 1st pet
NUM == 2; Past 2nd pet
NUM == 3; Past 3rd pet
NUM == 4; At 4th pet
NUM == 5; On way back from 4th pet
NUM == 6; At 3rd pet
NUM == 7; At 2nd pet
NUM == 8; At 1st pet
*/
void mainStart()
{
  while(true) {

    // pickup(ARM_LEFT);
    ArmPID(ANGLE, ARM_LEFT);
    ArmPID(ANGLE, ARM_CENTRE);

    if(startbutton())
    {
      delay(100);
      if(startbutton()){ return; }
    }
  }
}

/*
=================
== ARM CONTROL ==
=================
*/

/*
Sets the position of a servo to the specified angle
Params:
Servo - servo we want to control
Angle - sets the servo to this angle
*/
void setServo(int servo, int angle)
{
  if(servo == 0) {RCServo0.write(angle);}
  else if(servo == 1) {RCServo1.write(angle);}
  else {RCServo2.write(angle);}
}

/*
The arm moves up and to the centre of the box and then releases the pet
into the box
*/
void dropoff()
{
  ArmPID(HEIGHT, ARM_UP);
  ArmPID(ANGLE, ARM_CENTRE);
  setServo(SERVO_PLATE, 90);
  while(digitalRead(SWITCH_PLATE) == LOW) {}
  delay(500);
  setServo(SERVO_PLATE, 0);
}

/*
The arm will move to either the right or the left and then moves down.
The arm will then dropoff the pet in the box.
Params:
Side - the side of the robot we want to pick up on
*/
void pickup(int side)
{
  int angle = 0;
  int attempt = 0;
  ArmPID(HEIGHT, ARM_UP);
  while (digitalRead(SWITCH_PLATE) == HIGH && attempt < 3) {
    switch (attempt) {
      case 0:
        angle = side;
        break;
      case 1:
        angle = side - SHIFT;
        break;
      case 2:
        angle = side + SHIFT;
        break;
      default:
        angle = side;
    }
    ArmPID(ANGLE, angle);
    ArmPID(HEIGHT, ARM_DOWN);
    ArmPID(HEIGHT, ARM_HOR);
    attempt++;
  }
  dropoff();
}

/*
=============
== ARM PID ==
=============
*/

// TODO: TUNE THIS CODE SO IT IS MORE RELIABLE

/*
PID control that operates on the homebrew servos.
For reference:
  P - value decreases as it gets closer to the deadband
  D - acts as the damping force as we move closer to the deadband
  I - helps push us into the deadband if the location is just outside
Params:
  dim - specifies which motor to use, either the horizontal (angle) or the vertical (hieght)
  pos - the position to set the arm too
*/

void ArmPID(int dim, int pos)
{
  //Set variables
  int P_gain = 0;
  int I_gain = 0;
  int max_speed = 0;
  int maxI = 0;
  int MOTOR = 0;
  int PIN = 0;
  int D_gain = 0;
  int deadband = 0;
  bool high_pet = false;
  bool low_pet = false;
  int cur_angle = 0;

  //Height
  if(dim == HEIGHT) {
    cur_angle = analogRead(POTENTIOMETER_CRANE_ANGLE);
    P_gain = P_HEIGHT;
    D_gain = D_HEIGHT;
    I_gain = I_HEIGHT;
    max_speed = SPEED_HEIGHT;
    maxI = I_MAX_HEIGHT;
    MOTOR = MOTOR_CRANE_HEIGHT;
    PIN = POTENTIOMETER_CRANE_HEIGHT;
    deadband = DEADBAND_HEIGHT;
    if (pos == ARM_DOWN) {
      if ((ARM_RIGHT - (SHIFT + DEADBAND_ANGLE)) <= cur_angle && cur_angle <= (ARM_RIGHT + (SHIFT + DEADBAND_ANGLE))){high_pet = true;}
      else {low_pet = true;}
    }
  }
  //Angle
  else {
    P_gain = P_ANGLE;
    I_gain = I_ANGLE; // I_angle = 1
    D_gain = D_ANGLE;
    max_speed = SPEED_ANGLE;
    maxI = I_MAX_ANGLE;
    MOTOR = MOTOR_CRANE_ANGLE;
    PIN = POTENTIOMETER_CRANE_ANGLE;
    deadband = DEADBAND_ANGLE;
   }

  // Variables
  int pot = 0;
  int count = 0;
  int target = 0;

  // PID variables
  float proportional = 0;
  float integral = 0;
  float derivative = 0;
  float compensator = 0;

  // Errors
  float error = 0;
  float last_error = 0;

  // Timing
  long start_pid = millis();

  unsigned long last_integral_update_ms = 0;
  const unsigned int integral_update_delay_ms = 5;

  while(true){

    // only print every 500 iterations
    if (count > 500){
          P_gain =  analogRead(7) / 100.0;
          D_gain = analogRead(6);
          LCD.clear(); LCD.home();
          LCD.print("P "); LCD.print(P_gain); LCD.print(" D ");LCD.print(D_gain); LCD.setCursor(0,1);
          LCD.print(error);
          // LCD.print(pos);
          // LCD.setCursor(0,1);
          // LCD.print(pot);

          count = 0;
    }
    count++;

    pot = analogRead(PIN);

    error = ((pot - pos) / 10.0);
    if(error < 0) {error = error -3;}
    else {error = error + 3;}

    if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
      // error = 0;
      target++;
    }
    else { target = 0; }

    if(pot - pos == 0) {error = 0;}

    proportional = P_gain * error;
    derivative = ( (float) D_gain )* (error - last_error);
    integral = I_gain * (error) / 100.0 + integral;

    // handling integral gain
    if ( integral > maxI) { integral = maxI;}
    if ( integral < -maxI) { integral = -maxI;}
    if( error == 0) { integral = 0; }

    compensator = proportional + integral + derivative;

    // setting max speed for the small motor

    if( compensator > max_speed) compensator = max_speed;
    if( compensator < -max_speed) compensator = -max_speed;

    motor.speed(MOTOR, compensator);

    last_error = error;

    if ( target > 1000 ) {
      return;
    }

    if(digitalRead(SWITCH_PLATE) == LOW && pos == ARM_DOWN) {return;}
  }
}

/*
==========
== MENU ==
==========
*/

/*
Control code for the menu where we can adjust values to tune PID control
(only for tape following)
*/
void Menu()
{
  LCD.clear(); LCD.home();
  LCD.print("Entering menu");
  delay(500);

  while (true) {
    /* Show MenuItem value and knob value */
    int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
    LCD.clear(); LCD.home();
    LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
    LCD.setCursor(0, 1);
    LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
    delay(100);

    /* Press start button to save the new value */
    if (startbutton())
    {
      delay(100);
      if (startbutton())
      {
        menuItems[menuIndex].Value = knob(7);
        menuItems[menuIndex].Save();
        delay(250);
      }
    }

    /* Press stop button to exit menu */
    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      {
        LCD.clear(); LCD.home();
        LCD.print("Leaving menu");
        delay(500);
        return;
      }
    }
  }
}
