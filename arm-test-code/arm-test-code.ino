#include <avr/EEPROM.h>
#include <phys253.h>
#include <LiquidCrystal.h>

//volatile unsigned int NUM = 0;
int NUM = 0;
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
MenuItem ProportionalGainTape = MenuItem("P-gain Tape");
MenuItem DerivativeGain   = MenuItem("D-gain Tape");
MenuItem IntegralGain     = MenuItem("I-gain Tape");
MenuItem ThresholdTape    = MenuItem("Thresh Tape");
MenuItem ProportionalGainIR = MenuItem("P-gain IR");
MenuItem ThresholdIR      = MenuItem("Thresh IR");
MenuItem menuItems[]      = {Speed, ProportionalGainTape, DerivativeGain, IntegralGain, ThresholdTape, ProportionalGainIR, ThresholdIR};

void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
}

//TINAH Inputs

//Motor   
int MOTOR_LEFT = 2; //PWM output for left motor
int MOTOR_RIGHT = 3; //PWM output for right motor
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
int SWITCH_PLATE = 1; //Switch to see if pet is on plate

//Other constants
int MAX_ANALOG = 1023;
int SPEED_HEIGHT = 90;
int SPEED_ANGLE = 70;
int P_HEIGHT = 20;
int P_ANGLE = 12;
int I_HEIGHT = 24;
int I_ANGLE = 1;
int I_MAX_HEIGHT = 150;
int I_MAX_ANGLE = 150;
int ARM_UP = 950;
int ARM_DOWN = 700;
int ARM_PICKUP = 600;
int ARM_LEFT = 250;
int ARM_CENTRE = 500;
int ARM_RIGHT = 730;
int DEADBAND = 15;
int TURNAROUND_DELAY = 300;

//For reference
int HEIGHT = 1;
int ANGLE = 2;

void loop()
{ 
  stopDrive();
  setServo(SERVO_PLATE, 0);
  
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
      mainStart();  
    } 
  }
}
 
void mainStart()
{ 
  LCD.clear();
  while(true) {    
      ArmPID(HEIGHT,ARM_UP);
      ArmPID(ANGLE,ARM_CENTRE);
      pickup(ARM_RIGHT,ARM_UP);
  }
}

void PIDTape()
{  
  //Variables
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet = 0; //Value of pet qrd
  int error = 0; //Current error
  int last_error = 0; //Previous error
  int recent_error = 0; //Recent error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0; //Compensation
  
  int spd = (int)((float)S*((float)255/(float)MAX_ANALOG));
  
  int WAIT_TIME = 300;
  long start_time = 0;
  int count = 0;
  
  //PID loop
  while (true)
  {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT);
    qrd_right = analogRead(QRD_RIGHT);
    qrd_pet = analogRead(QRD_PET_FRONT);
    
    //Check if pet needs picking up
    if(count == 0){start_time = millis(); last_error = 5; count++;}
    if ((millis() - start_time) > WAIT_TIME) {
      if(qrd_pet > THRESHOLD) {
        NUM++;
        if(NUM == 1 || NUM == 2 || NUM == 3) {break;}
      }
    }
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
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
    compensation = proportional + derivative;
    
    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
  }
}

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

void stopDrive() {
  motor.speed(MOTOR_LEFT, 0);
  motor.speed(MOTOR_RIGHT, 0);
}

void moveToPet() {
  int THRESHOLD = menuItems[4].Value;
  LCD.clear(); LCD.home(); LCD.print(QRD_PET_BACK);
  stopDrive();
  delay(4000);
  motor.speed(MOTOR_LEFT, 150);
  motor.speed(MOTOR_RIGHT, 150);
  while(analogRead(QRD_PET_BACK) < THRESHOLD) {}
  stopDrive();
}

void moveBack() {
  int THRESHOLD = menuItems[4].Value;
  LCD.clear(); LCD.home(); LCD.print(QRD_LEFT);
  stopDrive();
  delay(4000);
  motor.speed(MOTOR_LEFT, -150);
  motor.speed(MOTOR_RIGHT, -150);
  while(analogRead(QRD_LEFT) < THRESHOLD) {}
  stopDrive();
}

void setServo(int servo, int angle) {
  if(servo == 0) {RCServo0.write(angle);}
  else if(servo == 1) {RCServo1.write(angle);}
  else {RCServo2.write(angle);}
}

void dropoff() {
  ArmPID(HEIGHT, ARM_UP);
  ArmPID(ANGLE, ARM_CENTRE);
  setServo(SERVO_PLATE, 90);
  while(digitalRead(SWITCH_PLATE) == LOW) {}
  delay(500);
  setServo(SERVO_PLATE, 0);
}

void pickup(int side, int height) {  
  ArmPID(HEIGHT, ARM_UP);
  ArmPID(ANGLE, side);
  ArmPID(HEIGHT, ARM_DOWN);
  dropoff();
}

void turnAround() {
  int THRESHOLD = menuItems[4].Value;
  LCD.clear(); LCD.home(); LCD.print("TURNING");
  delay(2000);
  motor.speed(MOTOR_LEFT, -100);
  motor.speed(MOTOR_RIGHT, 100);
//  delay(TURNAROUND_DELAY);
  while(analogRead(QRD_LEFT) < THRESHOLD) {}
  stopDrive();
}

void ArmPID(int dim, int pos)
{  
   //Set variables
    int P_gain;
    int I_gain;
    int max_speed;
    int maxI;
    int MOTOR;
    int PIN;
    int D_gain = 0;
   
   //Height
   if(dim == HEIGHT) {
      P_gain = P_HEIGHT;
      D_gain = 0;
      I_gain = I_HEIGHT;
      max_speed = SPEED_HEIGHT;
      maxI = I_MAX_HEIGHT;
      MOTOR = MOTOR_CRANE_HEIGHT;
      PIN = POTENTIOMETER_CRANE_HEIGHT;
   }
   //Angle
   else {
      P_gain = P_ANGLE;
      I_gain = I_ANGLE;
      D_gain = 4;
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
   
   // STARTING THE TIMER
   int DELAY = 1500;
   
   LCD.clear(); 
   
   while(true){
     
       LCD.home();
       LCD.print(pos); LCD.setCursor(0,1);
       LCD.print(pot);
     
       pot = analogRead(PIN);
       
       error = (pot -pos) / 10.0;
       
       if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
           error = 0;
           target++;
       }
       else { target = 0; }
      
       proportional = P_gain * error;
       derivative = D_gain * (error - last_error);    
       integral = I_gain * error / 100.0 + integral;
       
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
          
       // Break the loop if the 
//       if(error == 0) {
//         target = millis();
//         while(pot <= ( pos + deadband ) && pot >= ( pos - deadband)){
//           if(millis() - target > DELAY) {return;}
//         }
//       }
       if ( target > 500 )
       {
         return;
       }
        
       if(digitalRead(SWITCH_PLATE) == LOW && pos == ARM_DOWN) {return;}
   }
}
