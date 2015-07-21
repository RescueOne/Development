
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
//    LCD.setCursor(0,0); LCD.print("Speed: ");
    LCD.setCursor(0,1);
    
    // Input pins
    int knob_pin = 7;
    int pot_pin = 4;
    
    // Variables
    int knob = 200;
    int pot = 200;
    int motor_speed;
    int count = 0;
    int target = 0;
    
    // PID variables
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    int P_gain;
    int I_gain;
    int D_gain;
    int dir;
    double compensator = 0;
    int maxI = 150;
    
    // Errors
    double error = 0;
    double last_error = 0;
    int sum_error = 0;
    
    // Setting values
    motor_speed = menuItems[0].Value;
    P_gain = menuItems[1].Value;
    D_gain = menuItems[2].Value;
    I_gain = menuItems[3].Value;
    
    double frac_error = 0;
    
    while(true){
      // (knob - 511.5)/2.0
        knob = 800;
        pot = analogRead(pot_pin);
        
//         if (pot > 1000 || pot < 600) { motor.speed(1, 0); continue;}
         
//        if (knob > 950) knob = 950;
//        if (knob < 600) knob = 700;

        
        //  int motor = (knob - 511.5)/2.0; // speed between 255 and -255
//        frac = ((double) pot - (double) knob) / ( (double) pot + (double) knob);
        // general PID logic

//        if( pot > knob-25 || )
//        {
//            error = (pot - knob) / 10.0;
//            target = 0;
//        }
//        else if( pot < knob+25)
//        {
//            error = (pot - knob) / 10.0;
//            target = 0;
//        }
//        else
//        {
//            error = 0;
//            target++;
//        }

        if( pot > knob ) {
            error = (pot - knob) / 10.0;
        }
        if( pot < knob ) {
            error = (pot - knob) / 10.0;
        }
        // pot <= ( knob + 20) && pot >= ( knob - 20)
        if( pot <= ( knob + 25 ) && pot >= ( knob - 25)) {
            error = 0;
            target++;
        }
        else
        {
            target = 0;
        }
        
        proportional = (double) P_gain * error;    
        integral =  ( (double) I_gain ) * error / 100.0 + integral;
        derivative = (double) D_gain * (error - last_error);
        
        if ( integral > maxI) { integral = maxI;}
        if ( integral < -maxI) { integral = -maxI;}
        
        if ( error == 0 ) integral = 0;

        compensator = proportional + derivative + integral;
//        + derivative + integral;
        
        // setting max speed so the TINAH doesn't turn off
        int max_speed = 150;
        if( compensator > max_speed) compensator = max_speed;
        if( compensator < -max_speed) compensator = -max_speed;
        
        motor.speed(1, compensator);

        last_error = error;
        
        if( count == 300)
         {
               LCD.clear(); LCD.home();
               LCD.print("prop:"); LCD.print(pot);
               LCD.setCursor(0, 1);
               LCD.print("int:"); LCD.print(integral);
               LCD.print(" der:"); LCD.print(derivative);
//LCD.print("INT: "); LCD.print(integral);
               count = 0;
         }
         count++;
         
         if ( target > 1000)
         {
             LCD.clear(); LCD.home();
            LCD.print("TARGET");
            delay(500);
         }
         if(stopbutton())
         {
               delay(100);
               if(stopbutton())
               {
                     motor.speed(1, 0);
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

