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
MenuItem Threshold        = MenuItem("Threshold");
MenuItem menuItems[]      = {Speed, ProportionalGain, Threshold};
 
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
  //Variables
  int IR_PIN = 0; //Pin for IR sensor
  int P = menuItems[1].Value; //Proportional gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[2].Value; //Threshold max IR value
  int irVal = 0; //Value of IR sensor
  int error = 0; //Current error
  int lastError = 0; //Previous error
  int proportional = 0; //Proportional control
  int compensation = 0; //The final compensation value
  int newDirection = 0; //-1 for left, 1 for right
  int oldDirection = 0; //-1 for left, 1 for right
  
  int spd = (int)((float)S*((float)255/(float)1023));
  
  //PID loop
  while (true)
  {
    
    //For debugging
    LCD.clear(); LCD.home();
    LCD.print(error); LCD.print("  "); LCD.print(oldDirection);
    
    //Read QRD's
    irVal = analogRead(IR_PIN);
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
    error = THRESHOLD - irVal;
    
    //Was going right
    if (oldDirection > 0) {
      //Going in wrong direction
      if (lastError < error) {newDirection = -1;}
      //Going right direction
      else {newDirection = 1;}
    //Was going left
    } else {
      //Going wrong direction
      if (lastError < error) {newDirection = 1;} 
      //Going right direction
      else {newDirection = -1;}
    }
    
    //Proportional control
    proportional = P*error;
    
    //Compensation
    compensation = proportional*newDirection;
    
    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
    
    oldDirection = newDirection;
    lastError = error;
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
