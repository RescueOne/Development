#include <avr/EEPROM.h>
#include <phys253.h>          
#include <LiquidCrystal.h>

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
		Name 		  = name;
		Value         = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

// menu initialization
uint16_t MenuItem::MenuItemCount = 0;
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem Threshold        = MenuItem("Threshold");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain, Threshold};
 
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
int MOTOR_LEFT = 2; //PWM output for left motor
int MOTOR_RIGHT = 3; //PWM output for right motor
int MOTOR_VERTICAL = 0; //Motor for arm height
int MOTOR_HORIZONTAL = 1; //Motor for arm angle

//Analog
int QRD_RIGHT = 0; //Right QRD for tape following
int QRD_LEFT = 1; //Left QRD for tape following
int QRD_PET_BACK = 2; //QRD for locating pets back
int QRD_PET_FRONT = 3; //QRD for locating pets front
int POT_VERTICAL = 5; //Rotary potentiometer for crane arm
int POT_HORIZONTAL = 4; //Rotary potentiometer for crane arm

//Servo
int SERVO_PLATE = 0; //Servo to drop pet

//Digital
int SWITCH_PLATE = 1; //Switch to see if pet is on plate

/*
===============
== CONSTANTS ==
===============
*/

int PET_NUM = 0; // represents the location of the pet we are at

int MAX_ANALOG = 1023;

// Locations
int RIGHT = 750;
int MIDDLE = 500;
int UP = 920;
int DOWN = 700;

/*
=================
== HOME SCREEN ==
=================
*/

void loop()
{ 
	stopDrive();
	ResetPosition();

	LCD.clear(); LCD.home();
	LCD.print("Start: Menu");
	LCD.setCursor(0, 1);
	LCD.print("Stop: PID");
	delay(100);
 
	if (startbutton())
	{
		delay(100);
		if (startbutton()) { Menu(); }
	}

	if (stopbutton())
	{
		delay(100);
		if (stopbutton()) { mainLoop(); } 
	}
}

/*
===============
== MAIN LOOP ==
===============
*/

void mainLoop()
{
	// set arm to fit through the door then start tape following
	if (PET_NUM == 0) {
		ResetPosition();
	    PIDTape();
	}
	// pick up 3 pets along the course
	if (PET_NUM == 1 || PET_NUM == 2 || PET_NUM == 3 || PET_NUM == 4) 
	{
	    // moveToPet();
	    PickupAndDrop();
	    PIDTape();
  	}
  	// hopefully hit the 4th pet and continue driving
  	// if (PET_NUM == 4) {
  	// }
  	// Switch to IR
	if (PET_NUM == 5) { 
		stopDrive(); 
	}
}

/*
====================
== TAPE FOLLOWING ==
====================
*/

void PIDTape()
{  
  LCD.clear(); LCD.home();
  LCD.print("Following");
  
  //Variables
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrdLeft = 0; //Value of left qrd
  int qrdRight = 0; //Value of right qrd
  int qrdPet = 0; //Value of pet qrd
  int error = 0; //Current error
  int lastError = 0; //Previous error
  int recentError = 0; //Recent error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int durationRecent = 0; //number of loops on recent error
  int durationLast = 0; //number of loops on last error
  int compensation = 0; //Compensation
  
  int spd = (int)((float)S*((float)255/(float)MAX_ANALOG));
  
  int WAIT_TIME = 500;
  int startTime = millis();
  int count = 0;
  
  //PID loop
  while (true)
  {
    //Read QRD's
    qrdLeft = analogRead(QRD_LEFT);
    qrdRight = analogRead(QRD_RIGHT);
    qrdPet = analogRead(QRD_PET_FRONT);
    
    //Check if pet needs picking up
    if(qrdPet > THRESHOLD) {
      PET_NUM++;
      if(PET_NUM == 1 || PET_NUM == 2 || PET_NUM == 3 || PET_NUM == 4) {break;}
    } 
    if(count == 0 && PET_NUM == 1){lastError = 5; count++;}
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
    //left on white
    if(qrdLeft < THRESHOLD){
      //right on white
      if(qrdRight < THRESHOLD){
        if(lastError < 0) {error = -5;LCD.setCursor(0,1);LCD.print("L2");}
        else {error = 5;LCD.setCursor(0,1);LCD.print("R2");}
      }
      //right on black
      else{error = -1;LCD.setCursor(0,1);LCD.print("L1");}
    }
    //left on black
    else{
      //right on white
      if(qrdRight < THRESHOLD){error = 1;LCD.setCursor(0,1);LCD.print("R1");}
      //right on black
      else{error = 0;LCD.setCursor(0,1);LCD.print("CE");}
    }
    
    //determine control factors
    
    //Proportional control
    proportional = P*error;
    
    //Derivative
    if(error != lastError){
      recentError = lastError;
      durationRecent = durationLast;
      lastError = error;
      durationLast = 1;
    }
    else {
      durationLast++;
    }
    derivative = (int)(((float)D*(float)(error - recentError))/((float)(durationRecent + durationLast)));
    
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

void ArmPID(int pos, int motor_pin, int pot_pin)
{
    LCD.clear();  LCD.home();

    int P_gain;
    int I_gain;
    int D_gain;
    int maxI = 50;
    int max_speed = 0;

    // setting PID constants to be respective to the motor
    if( motor_pin == MOTOR_VERTICAL )
    {
        P_gain = 10;
        I_gain = 24;
        D_gain = 0;
        max_speed = 150;
        maxI = 150;
    }
    else // horizontal motor
    {
        P_gain = 2;
        I_gain = 1;
        D_gain = 4;
        max_speed = 70;
        maxI = 150;
    }
    
    // Variables
    int pot = 0;
    int count = 0;
    int target = 0;
    int deadband = 25; // will never be exactly in the spot we want but in a range
    // NOTE: working deadband was 35
    
    // PID variables
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double compensator = 0;
   
    // Errors
    double error = 0;
    double last_error = 0;
    
    // STARTING THE TIMER
    int start = millis();
    int DELAY = 2000;
    
    // cap the amount of time this loop can continue
    //  (millis() - start) <= DELAY
    while(true){

        pot = analogRead(pot_pin);
        
        // stopping the motor from moving into restricted areas
        // was not working, should test more
        // if( motor_pin == MOTOR_VERT) {
        //    if (pot > 1000 || pot < 600) { motor.speed(motor_pin, 0); continue;}
        // }
        // else
        // { 
        //    if (pot > 900 || pot < 100) { motor.speed(motor_pin, 0); continue;}
        // }
        
        // error handling
        error = (pot -pos) / 10.0;
        
        if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
            error = 0;
            target++;
        }
        else { target = 0; } //sometimes the arm was getting caught on the deadband line and this wasn't working
       
        proportional = P_gain * error;    
        integral = I_gain * error / 100.0 + integral;
        derivative = D_gain * (error - last_error);
        
        // capping the integral gain
        if ( integral > maxI ) { integral = maxI;}
        if ( integral < -maxI ) { integral = -maxI;}
        if ( error == 0 ) { integral = 0; }

        compensator = proportional + derivative + integral;
        
        // setting max speed for the small motor
        if( compensator > max_speed) compensator = max_speed;
        if( compensator < -max_speed) compensator = -max_speed;
        
        motor.speed(motor_pin, compensator);

        last_error = error;
        
        // print every 300 iterations
        if( count == 300)
         {
               LCD.clear(); LCD.home();
               LCD.print("POT: "); LCD.print(pot);
               LCD.setCursor(0, 1);
               LCD.print("POS: "); LCD.print(pos);
               count = 0;
         }
         count++;
         
         // Break the loop if the pet has been picked up
//         if(digitalRead(SWITCH_PLATE) == LOW) { break; }
           
         // Break the loop if the 
         if(target > 1000)
         {
               delay(100);
               if(target > 1000)
               {
                     return;
               }
         }
    }
}

/*
===================
== MOTOR CONTROL ==
===================
*/

// Stop the motors
void stopDrive() 
{
	motor.speed(MOTOR_LEFT, 0);
	motor.speed(MOTOR_RIGHT, 0);
}

/*
=====================
== ARM POSITIONING ==
=====================
*/

// Moves the arm to pick up the pet and releases it over the bucket
void PickupAndDrop() 
{
    SetArm(RIGHT);
    SetArm(DOWN);
    SetArm(UP);
    SetArm(MIDDLE);
    Release();
    ResetPosition();
}

// Set the arm location based on the given position
void SetArm(int loc)
{
	if ( loc == RIGHT || loc == MIDDLE) { }
	if ( loc == UP || loc == DOWN) { }
}

// Resets the position of the arm to middle and up
void ResetPosition()
{
    SetArm(UP); 
    SetArm(MIDDLE);
    stopDrive();
}

// Releases the pet with the servo on the plate
void Release()
{
	setServo(SERVO_PLATE, 90);
  	//while(digitalRead(SWITCH_PLATE) == LOW) {}
  	delay(500);
  	setServo(SERVO_PLATE, 0);
}

// Sets the position of the given servo to the given angle
void setServo(int servo, int angle) 
{
  	if(servo == SERVO_PLATE) {RCServo0.write(angle);}
}

/*
==========
== MENU ==
==========
*/

void Menu()
{  
	LCD.clear(); LCD.home();
	LCD.print("Entering menu");
	delay(500);
 
	while (true)
	{
		/* Show MenuItem value and knob value */
		int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
		LCD.clear(); LCD.home();
		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		int val = knob(7);
		// caps all the values between 0 and 255
        // val = (val / 1023.0) * 255;
		LCD.print("Set to "); LCD.print(val); LCD.print("?");
		delay(100);
 
		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			if (startbutton())
			{
                menuItems[menuIndex].Value = val;
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

