
#include <avr/EEPROM.h>
#include <phys253.h>          
#include <LiquidCrystal.h>

/*
*  Creates a "map" with the location of an EEPROM address
*  EEPROM = TINAH storage
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

/* Creating the Menu Items */
uint16_t MenuItem::MenuItemCount = 0;
// Add the menu items here
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain};

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600) ;
  
}

void loop()
{
	LCD.clear(); LCD.home();
	LCD.print("Start+stop=menu");
	LCD.setCursor(0, 1);
	LCD.print("Start=run");
	delay(100);
 
	if (startbutton() && stopbutton())
	{
		delay(100);
		if (startbutton())
		{
			Menu();
		}
	}
        if (startbutton())
        {
                LCD.clear(); LCD.home();
      	        LCD.print("Stop = exit!");
      
                PID();

                motor.speed(3,0);
        }
        if (stopbutton())
        {
                LCD.clear(); LCD.home();
      	        LCD.print("Start = exit!");
                
        }
}

/*
*  Function for Servo Control
*/
void SetServo() {
     while(!startbutton())
     {
           LCD.clear();  LCD.home();
           LCD.setCursor(0,0); LCD.print("Servo Angle: ");
           LCD.setCursor(0,1);
          
           int knob_pin = 7;
          
           int angle = ( analogRead(knob_pin) / 1023.0) * 180;
           RCServo1.write(angle) ;
           LCD.print(angle);
           delay(100); 
     }
 
}

/*
* Function for PID control
*/
void PID() {
    
    // Set the screen to be ready to print
    LCD.clear();  LCD.home();
    LCD.setCursor(0,0); LCD.print("Speed: ");
    LCD.setCursor(0,1);
    
    // Input pins
    int knob_pin = 7;
    int pot_pin = 5;
    
    // Variables
    int knob = 200;
    int pot = 200;
    int motor_speed;
    int count = 0;
    
    // PID variables
    int proportional = 0;
    int integral = 0;
    int derivative = 0;
    int P_gain;
    int I_gain;
    int D_gain;
    int dir;
    double compensator = 0;
    
    // Errors
    int error = 0;
    int last_error = 0;
    int sum_error = 0;
    
    // Setting values
    motor_speed = menuItems[0].Value;
    P_gain = menuItems[1].Value;
    D_gain = menuItems[2].Value;
    I_gain = menuItems[3].Value;
    
    double frac_error = 0;
    
    while(true){
      // (knob - 511.5)/2.0
        knob = analogRead(knob_pin);
        pot = analogRead(pot_pin);
        
        if (knob > 950) knob = 950;
        if (knob < 700) knob = 700;
        
        //  int motor = (knob - 511.5)/2.0; // speed between 255 and -255
//        frac = ((double) pot - (double) knob) / ( (double) pot + (double) knob);
        // general PID logic

        if( pot > knob ) {
            error = (pot - knob) / 10.0;
        }
        if( pot < knob ) {
            error = (pot - knob) / 10.0;
        }
        // pot <= ( knob + 20) && pot >= ( knob - 20)
        if( pot <= ( knob + 35 ) && pot >= ( knob - 35)) {
            error = 0;
        }
        
//        frac_error = ( (double) error ) / ((double) knob + (double) pot);
        
        proportional = P_gain * error;    
//        integral = I_gain * sum_error;
        derivative = D_gain * (error - last_error);
        
        compensator = proportional + derivative;
//        + derivative + integral;
        
        // setting max speed so the TINAH doesn't turn off
        int max_speed = 150;
        if( compensator > max_speed) compensator = max_speed;
        if( compensator < -max_speed) compensator = -max_speed;
        
        motor.speed(3, compensator);

        last_error = error;
        
        if( count == 300)
         {
               LCD.clear(); LCD.home();
               LCD.print("POT: "); LCD.print(pot);
               LCD.setCursor(0, 1);
               LCD.print("KNOB: "); LCD.print(knob);
//LCD.print("Error: "); LCD.print(error);
               count = 0;
         }
         count++;
         
         if(stopbutton())
         {
               delay(100);
               if(stopbutton())
               {
                     break;
               }
         }
    }
}

// Opens the menu
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
                val = (val / 1023.0) * 255;
		LCD.print("Set to "); LCD.print(val); LCD.print("?");
		delay(100);
 
		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			if (startbutton())
			{
                                // Can max motor speed at +-700 so capping the speed at 700
                                if (menuIndex == 0)
                                {
                                        if ( val > 700) {
                                                LCD.clear(); LCD.home();
                                                LCD.print("Speed capped at 700.");
                                                val = 700;
                                                delay(250);
                                        }
                                }
                                
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

