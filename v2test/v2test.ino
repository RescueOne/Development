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
int MOTOR_LEFT = 3; //PWM output for left motor
int MOTOR_RIGHT = 2; //PWM output for right motor
int MOTOR_CRANE_HEIGHT = 1; //Motor for arm height
int MOTOR_CRANE_ANGLE = 0; //Motor for arm angle

//Analog
int QRD_LEFT = 1; //Left QRD for tape following
int QRD_RIGHT = 0; //Right QRD for tape following
int QRD_PET_BACK = 2; //QRD for locating pets back
int QRD_PET_FRONT = 3; //QRD for locating pets front
int POTENTIOMETER_CRANE_HEIGHT = 5; //Rotary potentiometer for crane arm
int POTENTIOMETER_CRANE_ANGLE = 4; //Rotary potentiometer for crane arm

//Servo
int SERVO_PLATE = 0; //Servo to drop pet

//Digital
int SWITCH_PLATE = 2; //Switch to see if pet is on plate
int ROT_LEFT = 0; //Rotary encoder for left wheel
int ROT_RIGHT = 1; //Rotary encoder for right wheel

/*
===============
== CONSTANTS ==
===============
*/

//For Arm PID
int MAX_ANALOG = 1023;
int SPEED_HEIGHT = 150;
int SPEED_ANGLE = 70;
int P_HEIGHT = 20;
int P_ANGLE = 12;
int I_HEIGHT = 24;
int I_ANGLE = 1;
int I_MAX_HEIGHT = 150;
int I_MAX_ANGLE = 150;
int D_HEIGHT = 0;
int D_ANGLE = 4;
int ARM_UP = 950;
int ARM_HOR = 830;
int ARM_DOWN = 700;
int ARM_PICKUP = 600;
int ARM_LEFT = 250;
int ARM_CENTRE = 500;
int ARM_RIGHT = 700;
int DEADBAND = 15;
int PET_QRD_THRESHOLD = 250;

//For reference
int HEIGHT = 1;
int ANGLE = 2;

//For rot encoder
float DIST_PER_TAPE = 6.3; //Distance wheel moves per tape hit (cm)
float LENGTH_OF_AXLE = 23.7; //Length of axle (cm)
int TURNS_LEFT = 0;
int TURNS_RIGHT = 0;
int prev_enc_left = 0;
int prev_enc_right = 0;
int cur_enc_left = 0;
int cur_enc_right = 0;

/*
=================
== HOME SCREEN ==
=================
*/

void loop()
{
  stopDrive();
  setServo(SERVO_PLATE, 0);

  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Stop: PID");
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
void mainStart() {
  LCD.clear();
  while(true) {
    // at the start, sets the arm to fit in the door then runs PID code
    if (NUM == 0) {
      ArmPID(HEIGHT,ARM_UP);
      ArmPID(ANGLE,ARM_CENTRE);
      ArmPID(HEIGHT,ARM_HOR);
      PIDTape();
    }
    if (NUM == 4) {
      LCD.setCursor(0,0); LCD.print(NUM);
      stopDrive();
      turnAround();
      NUM++;
      PIDTape();
    }
    if (NUM == 6 || NUM == 7 || NUM == 8) {
      moveToPet();
      ArmPID(HEIGHT,ARM_UP);
      pickup(ARM_LEFT);
      moveBack();
      PIDTape();
    }
  }
}

/*
===================
== MOTOR CONTROL ==
===================
*/

/*
Stops the drive motors.
*/
void stopDrive() {
  motor.speed(MOTOR_LEFT, 0);
  motor.speed(MOTOR_RIGHT, 0);
}

/*
When the front pet QRD senses the perpindicular tape, the robot drives forward
until the back pet QRD senses the perpindicular tape.
*/
void moveToPet() {
  motor.speed(MOTOR_LEFT, 150);
  motor.speed(MOTOR_RIGHT, 150);
  if(NUM == 5) {delay(500);}
  if(NUM == 6 || NUM == 7) {delay(700);}
  //  while(analogRead(QRD_PET_BACK) < PET_QRD_THRESHOLD) {}
  stopDrive();
}

/*
After a pet has been picked up, the robot will move backwards until QRD_LEFT
is back on the tape
*/
void moveBack() {
  motor.speed(MOTOR_LEFT, -150);
  motor.speed(MOTOR_RIGHT, -150);
  while(analogRead(QRD_LEFT) < PET_QRD_THRESHOLD) {}
  stopDrive();
}

/*
After we pick up the 4th pet, we turn around until QRD_LEFT is back on the tape
*/
void turnAround() {
  int THRESHOLD = menuItems[4].Value;
  motor.speed(MOTOR_LEFT, -100);
  motor.speed(MOTOR_RIGHT, 100);
  //  delay(TURNAROUND_DELAY);
  while(analogRead(QRD_LEFT) < THRESHOLD) {}
  stopDrive();
}

/*
Move the robot to a position based on a given angle and distance
+ve angle turns left
-ve angle turns right
*/
void moveTo(int angle, float distance) {
  //First change angle
  int angleSpeed = 100;
  int linSpeed = 150;
  int angleTurns = ceil((abs(angle)*((LENGTH_OF_AXLE*PI)/360))/DIST_PER_TAPE);
  int linTurns = ceil(abs(distance)/DIST_PER_TAPE);
  bool leftDone = false;
  bool rightDone = false;
  TURNS_RIGHT = 0; TURNS_LEFT = 0;
  if (angle > 0) {
    motor.speed(MOTOR_LEFT, -1*angleSpeed);
    motor.speed(MOTOR_RIGHT, angleSpeed);
  } else {
    motor.speed(MOTOR_LEFT, angleSpeed);
    motor.speed(MOTOR_RIGHT, -1*angleSpeed);
  }
  // LCD.clear(); LCD.home(); LCD.print("Turning");
  while(leftDone == false || rightDone == false) {
    checkEnc();
    // LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    if(TURNS_LEFT == angleTurns) {motor.speed(MOTOR_LEFT,0); leftDone = true;}
    if(TURNS_RIGHT == angleTurns) {motor.speed(MOTOR_RIGHT,0); rightDone = true;}
  }
  TURNS_LEFT = 0; TURNS_RIGHT = 0;
  leftDone = false; rightDone = false;
  if (distance > 0) {motor.speed(MOTOR_LEFT,linSpeed); motor.speed(MOTOR_RIGHT, linSpeed);}
  else {motor.speed(MOTOR_LEFT,-1*linSpeed); motor.speed(MOTOR_RIGHT,-1*linSpeed);}
  // LCD.clear(); LCD.home(); LCD.print("Moving");
  while(leftDone == false || rightDone == false) {
    checkEnc();
    // LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    if(TURNS_LEFT == linTurns) {motor.speed(MOTOR_LEFT,0); leftDone = true;}
    if(TURNS_RIGHT == linTurns) {motor.speed(MOTOR_RIGHT,0); rightDone = true;}
  }
}

/*
Check encoders and update turns
*/
void checkEnc() {
  cur_enc_left = digitalRead(ROT_LEFT); cur_enc_right = digitalRead(ROT_RIGHT);
  if(prev_enc_left == HIGH && cur_enc_left == LOW) {
    TURNS_LEFT++;
  }
  if(prev_enc_right == HIGH && cur_enc_right == LOW) {
    TURNS_RIGHT++;
  }
  prev_enc_left = cur_enc_left; prev_enc_right = cur_enc_right;
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
  void setServo(int servo, int angle) {
    if(servo == 0) {RCServo0.write(angle);}
    else if(servo == 1) {RCServo1.write(angle);}
    else {RCServo2.write(angle);}
  }

/*
The arm moves up and to the centre of the box and then releases the pet
into the box
*/
void dropoff() {
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
  void pickup(int side) {
    ArmPID(HEIGHT, ARM_UP);
    ArmPID(ANGLE, side);
    ArmPID(HEIGHT, ARM_DOWN);
    dropoff();
  }

/*
========================
== TAPE FOLLOWING PID ==
========================
*/

// TODO: CLEAN UP MESSY CONTROL IF STATEMENTS

/*
PID control for tape following.
For reference:
  P - if too high can cause osscillations
  D - acts as damping
  */
void PIDTape() {
  //Variables
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet_front = 0; //Value of pet qrd
  int qrd_pet_back = 0; //Value of back pet QRD
  int error = 0; //Current error
  int last_error = 0;
  int recent_error = 0; //Recent error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0; //Compensation

  // converting S into a value that is <255
  int spd = (int)((float)S*((float)255/(float)MAX_ANALOG));

  int WAIT_TIME = 600;
  int WAIT_TIME_LONG = 3500;
  int WAIT_TIME_SHORT = 400;
  long start_time = 0;
  long start_time_2 = 0;
  int count = 0;

  //PID loop
  while (true) {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT);
    qrd_right = analogRead(QRD_RIGHT);
    qrd_pet_front = analogRead(QRD_PET_FRONT);

    if(count == 0){start_time = millis(); count++;}
    if((millis() - start_time) > WAIT_TIME) {
      if(qrd_pet_front > PET_QRD_THRESHOLD) {
        NUM++;
        count--;
        LCD.setCursor(0,0); LCD.print(NUM);
        if(NUM == 4 || NUM == 6 || NUM == 7 || NUM == 8) {return;}
      }
    }

    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */

    //left on white
    if(qrd_left < THRESHOLD) {
      //right on white
      if(qrd_right < THRESHOLD) {
        if(last_error < 0) {error = -5;LCD.setCursor(0,1);LCD.print("L2");}
        else {error = 5;LCD.setCursor(0,1);LCD.print("R2");}
      }
      //right on black
      else{error = -1;LCD.setCursor(0,1);LCD.print("L1");}
    }
    //left on black
    else {
      //right on white
      if(qrd_right < THRESHOLD){error = 1;LCD.setCursor(0,1);LCD.print("R1");}
      //right on black
      else{error = 0;LCD.setCursor(0,1);LCD.print("CE");}
    }

    //determine control factors

    //Proportional control
    proportional = P*error;

    //Derivative
    if(error != last_error) {
      recent_error = last_error;
      duration_recent = duration_last;
      last_error = error;
      duration_last = 1;
    }
    else {
      duration_last++;
    }
    derivative = (int)(((float)D*(float)(error - recent_error))/((float)(duration_recent + duration_last)));

    //Compensation
    compensation = proportional + derivative;

    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
  }
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

void ArmPID(int dim, int pos) {
  //Set variables
  int P_gain;
  int I_gain;
  int D_gain;
  int max_speed;
  int maxI;
  int MOTOR;
  int PIN;

  // Need to specify the constants P,I or D depending on the dimension
  //Height
  if(dim == HEIGHT) {
    P_gain = P_HEIGHT;
    I_gain = I_HEIGHT;
    D_gain = D_HEIGHT;
    max_speed = SPEED_HEIGHT;
    maxI = I_MAX_HEIGHT;
    MOTOR = MOTOR_CRANE_HEIGHT;
    PIN = POTENTIOMETER_CRANE_HEIGHT;
  }

  //Angle
  else {
    P_gain = P_ANGLE;
    I_gain = I_ANGLE;
    D_gain = D_ANGLE;
    max_speed = SPEED_ANGLE;
    maxI = I_MAX_ANGLE;
    MOTOR = MOTOR_CRANE_ANGLE;
    PIN = POTENTIOMETER_CRANE_ANGLE;
  }

  // Variables
  int pot = 0;
  int count = 0;
  long target = 0;
  int deadband = DEADBAND;

  // PID variables
  double proportional = 0;
  double integral = 0;
  double derivative = 0;
  double compensator = 0;

  // Errors
  double error = 0;
  double last_error = 0;

  while(true){
    pot = analogRead(PIN);

    error = (pot-pos) / 10.0;

    if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
      error = 0;
      target++;
    }
    else { target = 0; }

    proportional = P_gain * error;
    integral = I_gain * error / 100.0 + integral;
    derivative = D_gain * (error - last_error);

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

    if ( target > 500 ) {
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
void Menu() {
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
