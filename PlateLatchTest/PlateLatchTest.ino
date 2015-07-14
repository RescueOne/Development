////////////////////////////////////////////////
// TINAH Template Program - UBC Engineering Physics 253
// (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
/////////////////////////////////////////////////


#include <phys253.h>          
#include <LiquidCrystal.h>    


void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600) ;
}




void loop()
{
  if(startbutton()){RCServo2.write(180);LCD.home();LCD.print("Closed");}
  if(stopbutton()){RCServo2.write(90);LCD.home();LCD.print("Opened");}
}

