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

int MOTOR_LEFT = 2; //PWM output for left motor
int MOTOR_RIGHT = 3; //PWM output for right motor

void loop()
{
  int MAX_ANALOG = 1023;
  
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
      PID();  
    } 
  }
}
 
void PID()
{  
  LCD.clear(); LCD.home();
  LCD.print("Following");
  
  //Variables
  int QRD_LEFT_PIN = 1; //Analog pin for left QRD
  int QRD_RIGHT_PIN = 0; //Analog pin for right QRD
  int QRD_PET = 2;
  int MAX_INTEGRAL = 50; //Maximum integral term value
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int I = menuItems[3].Value; //Integral gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet = 0;
  int error = 0; //Current error
  int last_error = 0; //Previous error
  int recent_error = 0; //Recent error
  int total_error = 0; //Integral of the error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int integral = 0; //Integral control
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0; // 
  
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
    
      LCD.setCursor(0,1);LCD.print("PICKING UP PET");delay(3000); 
    
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
