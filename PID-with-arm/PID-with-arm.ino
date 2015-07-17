#include <avr/EEPROM.h>
#include <phys253.h>          
#include <LiquidCrystal.h>
 
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
 
uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem Threshold        = MenuItem("Threshold");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain, Threshold};
 
void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
}

/*
======================
== GLOBAL VARIABLES ==
======================
*/

// Pins
int MOTOR_VERT = 0; // For pulley motor
int MOTOR_HOR = 1; // Horizontal motor
int MOTOR_LEFT = 2; //PWM output for left motor
int MOTOR_RIGHT = 3; //PWM output for right motor
int QRD_RIGHT_PIN = 0; //Analog pin for right QRD
int QRD_LEFT_PIN = 1; //Analog pin for left QRD
int QRD_PET = 2; // For pet sensing when we find a pet
int SERVO_PLATE = 0; 
int POT_VERT = 5;
int POT_HOR = 4;

// Locations
int RIGHT = 800;
int MIDDLE = 500;
int UP = 950;
int DOWN = 650;

int reset = 0;
void loop()
{

  motor.speed(MOTOR_LEFT,0);
  motor.speed(MOTOR_RIGHT,0);
  
  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Stop: PID");
  delay(100);
 
  if (startbutton())
  {
    delay(100);
    if (startbutton())
    {
      Menu();
    }
  }

  if (stopbutton())
  {
    delay(100);
    if (stopbutton())
    {
      // Setting the arm to the central position so it can fit through the door
//      if( reset == 0)
//      {
          setServo(SERVO_PLATE, 0);
          ResetPosition();
          reset++;
//      }
      
//      PID();  
    } 
  }
}

/*
======================== 
== TAPE FOLLOWING PID ==
========================
*/

void PID()
{  
  LCD.clear(); LCD.home();
  LCD.print("Following");
  
  // Menu Variables
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int I = menuItems[3].Value; //Integral gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  
  // QRDs
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet = 0;
  
  // Error Handling
  int error = 0; //Current error
  int last_error = 0; //Previous error
  int recent_error = 0; //Recent error
  int total_error = 0; //Integral of the error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int integral = 0; //Integral control
  int MAX_INTEGRAL = 50; //Maximum integral term value
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0;
  
  int spd = (int)((float)S*((float)255/(float)1023));
  
  //PID loop
  while (true)
  {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT_PIN);
    qrd_right = analogRead(QRD_RIGHT_PIN);
    qrd_pet = analogRead(QRD_PET);
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
    if (qrd_pet > THRESHOLD) { 
    
      motor.speed(MOTOR_LEFT,0);
      motor.speed(MOTOR_RIGHT,0);
      
      // pickup pet
      LCD.setCursor(0,1);LCD.print("PICKING UP PET");
      PickupAndDrop();
      delay(1000); 
    
  }
    
    //left on white
    if(qrd_left < THRESHOLD){
      //right on white
      if(qrd_right < THRESHOLD){
        if(last_error < 0) {error = -5;LCD.setCursor(0,1);LCD.print("L2");}
        else {error = 5;LCD.setCursor(0,1);LCD.print("R2");}
      }
      //right on black
      else{error = -1;LCD.setCursor(0,1);LCD.print("L1");}
    }
    //left on black
    else{
      //right on white
      if(qrd_right < THRESHOLD){error = 1;LCD.setCursor(0,1);LCD.print("R1");}
      //right on black
      else{error = 0;LCD.setCursor(0,1);LCD.print("CE");}
    }
    
    //determine control factors
    
    //Proportional control
    proportional = P*error;
    
    //Integral error
    total_error += error;
    integral = I*total_error;
    if(integral > MAX_INTEGRAL) {integral = MAX_INTEGRAL;}
    if(integral < -MAX_INTEGRAL) {integral = -MAX_INTEGRAL;}
    
    //Derivative
    if(error != last_error){
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
    compensation = proportional + integral + derivative;
    
    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
  }
}

/*
=====================
== ARM POSITIONING ==
=====================
*/

void PickupAndDrop() 
{
    SetArmLoc(RIGHT);
    SetArmHeight(DOWN);
    SetArmHeight(UP);
    SetArmLoc(MIDDLE);
    Release();
    ResetPosition();
}

void SetArmHeight(int yloc)
{
    if( yloc == UP) { ArmPID(yloc, MOTOR_VERT, POT_VERT); }
    else { ArmPID(yloc, MOTOR_VERT, POT_VERT);}
}

void SetArmLoc(int xloc)
{
    if( xloc == RIGHT) { ArmPID(xloc, MOTOR_HOR, POT_HOR); }
    else { ArmPID(xloc, MOTOR_HOR, POT_HOR);}
}

void ResetPosition()
{
    SetArmHeight(UP); SetArmLoc(MIDDLE);
    motor.speed(MOTOR_HOR,0);
    motor.speed(MOTOR_VERT,0);
}

void Release()
{
  setServo(SERVO_PLATE, 90);
  //while(digitalRead(SWITCH_PLATE) == LOW) {}
  delay(1000);
  setServo(SERVO_PLATE, 0);
}

void setServo(int servo, int angle) 
{
  if(servo == SERVO_PLATE) {RCServo0.write(angle);}
}

/* returns the current position of the arm
  3 positions
  down left (or right) 
  up left
  over box
*/
//int DOWN_LEFT = 0;
//int UP_LEFT = 1;
//int OVER_BOX = 2;
//
//int CurrentPosition(int xloc, int yloc)
//{
//  
//}

/*
=====================
== ARM PID ==
=====================
*/

void ArmHorizontalPID(int pos, int motor_pin, int pot_pin)
{
// Set the screen to be ready to print
    LCD.clear();  LCD.home();
    LCD.setCursor(0,0); LCD.print("Speed: ");
    LCD.setCursor(0,1);
    
    // Variables
    int knob = 200;
    int pot = 200;
    int count = 0;
    int target = 0;
    
    // PID variables
    int proportional = 0;
    int integral = 0;
    int derivative = 0;
    int P_gain = 8;
    int I_gain = 1;
    int D_gain = 4;
    int maxI = 50;
    double compensator = 0;
    
    // Errors
    int error = 0;
    int last_error = 0;
    
    // Setting values
//    P_gain = menuItems[1].Value;
//    D_gain = menuItems[2].Value;
//    I_gain = menuItems[3].Value;
// temp off b/c it would coincide with tape following variables
    
    // setting PID gain to be respective to the motor
    if( motor_pin == MOTOR_VERT )
    {
        P_gain = 8;
        I_gain = 5;
        D_gain = 4;
    }
    else
    {
        P_gain = 8;
        I_gain = 1;
        D_gain = 4;
    }
    
    while(true){

        pot = analogRead(pot_pin);
        
        // stopping the motor from moving into restricted areas
        if( motor_pin == MOTOR_HOR )
        {
            if (pot > 900 || pot < 100) { motor.speed(motor_pin, 0); continue;}
        }
        else
        {
            if (pot > 1000 || pot < 550) { motor.speed(motor_pin, 0); continue;}
        }
        
        if( pot > pos ) {
            error = (pot - pos) / 10.0;
        }
        if( pot < pos ) {
            error = (pot - pos) / 10.0;
        }
        if( pot <= ( pos + 10 ) && pot >= ( pos - 10)) {
            error = 0;
            target++;
        }
       
        proportional = P_gain * error;    
        integral = I_gain * error + integral;
        derivative = D_gain * (error - last_error);
        
        // handling integral gain
        if ( integral > maxI) { integral = maxI;}
        if ( integral < -maxI) { integral = -maxI;}
        if( error == 0) { integral = 0; }

        compensator = proportional + derivative + integral;
        
        // setting max speed for the small motor
        int max_speed = 150;
        if( compensator > max_speed) compensator = max_speed;
        if( compensator < -max_speed) compensator = -max_speed;
        
        motor.speed(motor_pin, compensator);

        last_error = error;
        
        if( count == 300)
         {
               LCD.clear(); LCD.home();
               LCD.print("POT: "); LCD.print(pot);
               LCD.setCursor(0, 1);
               LCD.print("POS: "); LCD.print(pos);
               count = 0;
         }
         count++;
         
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

void ArmVerticalPID(int pos, int motor_pin, int pot_pin)
{

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
