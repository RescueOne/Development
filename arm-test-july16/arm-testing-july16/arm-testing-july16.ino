#include <avr/EEPROM.h>
#include <phys253.h>    
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

//class MenuItem
//{
//  public:
//  String Name;
//  uint16_t Value;
//  uint16_t* EEPROMAddress;
//  static uint16_t MenuItemCount;
//  MenuItem(String name)
//  {
//    MenuItemCount++;
//    EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
//    Name = name;
//    Value = eeprom_read_word(EEPROMAddress);
//  }
//    void Save()
//  {
//    eeprom_write_word(EEPROMAddress, Value);
//  }
//};
// 
//uint16_t MenuItem::MenuItemCount = 0;
///* Add the menu items here */
//MenuItem Speed            = MenuItem("Speed");
//MenuItem ProportionalGain = MenuItem("P-gain");
//MenuItem Up               = MenuItem("Up");
//MenuItem Down             = MenuItem("Down");
//MenuItem Pickup           = MenuItem("Pickup");
//MenuItem menuItems[]      = {Speed, ProportionalGain, Up, Down, Pickup};
//
void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
}

int MOTOR_LEFT = 2; //PWM output for left motor
int MOTOR_RIGHT = 3; //PWM output for right motor
int QRD_LEFT = 0; //Left QRD for tape following
int QRD_RIGHT = 1; //Right QRD for tape following
int QRD_PET = 2; //QRD for locating pets
int SWITCH_PLATE = 0; //Switch to see if pet is on plate
int SWITCH_FRONT = 2; //Switch on front of arm
int SERVO_CRANE = 1; //Servo for rotation of crane arm
int SERVO_PLATE = 2; //Servo to drop pet
int MOTOR_CRANE = 3; //Motor of arm movement
int POTENTIOMETER_CRANE = 4; //Rotary potentiometer for crane arm
int SERVO_FRONT = 2; //Servo for front arm
int KNOB1 = 6;
int KNOB2 = 7;

 //Constants
//      int DELAY = 2000;
      int MAX_COMPENSATOR = 150;
      
//      LCD.clear(); LCD.home();
//      LCD.print("Arm Height");
  
      // Variables
      int pot = 0;
//      int motor_speed = 0;
//      int target = 0;
      
      // PID variables
      int proportional = 0;
      int P_gain = 20;
      double compensator = 0;
      
      // Errors
      int error = 0;
//      int last_error = 0;
//      int sum_error = 0;
      


//Positions
//int MIDDLE = 105;

//volatile unsigned int NUM = 0;
//int MAX_ANALOG = 1023;

void loop()
{ 
  LCD.clear();LCD.home();LCD.setCursor(0,0);
  LCD.print("Start = Run");
  motor.speed(0,0);
  motor.speed(1,0);
  motor.speed(2,0);
  motor.speed(3,0);
  
   if (startbutton())
  {
    delay(100);
    if (startbutton())
    {
      test();
    }
  }
}
  

void test(){
  LCD.clear();LCD.home();LCD.setCursor(0,0);
  LCD.print("Stop = exit");
  while(true){
    
    setArmHeight(analogRead(KNOB1));
    setServo(SERVO_CRANE, analogRead(KNOB2));
    
    if(startbutton()){
      delay(100);
      if(startbutton()){
        setServo(SERVO_PLATE, 90);
        delay(500);
        setServo(SERVO_PLATE, 0);
      }
    }
    
    if (stopbutton()){
      delay(100);
      if (stopbutton()){
        break; 
      } 
    }
    
  
  
//  int pot = analogRead(POTENTIOMETER_CRANE); 
  
  
//  LCD.clear(); LCD.home();
//  LCD.print("Start: Menu");
//  LCD.setCursor(0, 1);
//  LCD.print(pot);
//  delay(100);

  
}
}
 
//void mainStart()
//{
//  LCD.clear(); LCD.home();
//  LCD.print("Left Down");
//  LCD.setCursor(0,1); LCD.print("Stop: Right Down");
//  while(!stopbutton()){
//    pickup(1,2);
//  }
//  LCD.clear(); LCD.home();
//  LCD.print("Right Down");
//  LCD.setCursor(0,1); LCD.print("Start: Right up");
//  while(!startbutton()){
//    pickup(2,2);
//  }
//  LCD.clear(); LCD.home();
//  LCD.print("Right Up");
//  LCD.setCursor(0,1); LCD.print("Stop: end cycle");
//  while(!stopbutton()){
//    pickup(2,1);
//  }
//}

//void Menu()
//{
//	LCD.clear(); LCD.home();
//	LCD.print("Entering menu");
//	delay(500);
// 
//	while (true)
//	{
//		/* Show MenuItem value and knob value */
//		int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
//		LCD.clear(); LCD.home();
//		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
//		LCD.setCursor(0, 1);
//		LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
//		delay(100);
// 
//		/* Press start button to save the new value */
//		if (startbutton())
//		{
//			delay(100);
//			if (startbutton())
//			{
//				menuItems[menuIndex].Value = knob(7);
//				menuItems[menuIndex].Save();
//				delay(250);
//			}
//		}
// 
//		/* Press stop button to exit menu */
//		if (stopbutton())
//		{
//			delay(100);
//			if (stopbutton())
//			{
//				LCD.clear(); LCD.home();
//				LCD.print("Leaving menu");
//				delay(500);
//				return;
//			}
//		}
//	}
//  }

//Sets a servo
void setServo(int servo, int angle) {
  if(servo == SERVO_CRANE) {RCServo0.write(angle); delay(100);}
  else if(servo == 2) {RCServo1.write(angle);}
  else {RCServo2.write(angle);}
}

////Picks up pet from side
//void pickup(int side, int height) {
//  if(height == 1) setArmHeight(1);
//  if(side == 1) setServo(SERVO_CRANE, 0);
//  else setServo(SERVO_CRANE, 180);
//  delay(1000);
//  setArmHeight(0);
//  dropoff();
//}

////Drops off pet in basket
//void dropoff() {
//  setArmHeight(1);
//  setServo(SERVO_CRANE, MIDDLE - 20);
//  delay(2000);
//  setServo(SERVO_PLATE, 90);
//  //while(digitalRead(SWITCH_PLATE) == LOW) {}
//  delay(1000);
//  setServo(SERVO_PLATE, 0);
//}

//Set the arm height, two settings
void setArmHeight(int height) {
     
      // Setting values
//      motor_speed = 100;
//      P_gain = 20;
      
//      double frac_error = 0;
      
//      LCD.setCursor(0,1); LCD.print(height);
         
//      while(true){
    
          pot = analogRead(POTENTIOMETER_CRANE);
          
          if( pot <= height+35 && pot >= height-35){
            error=0;
          }
          else if( pot > height + 35){
            error = pot / 10.0;
          }
          else{
            error = (pot - height) / 10.0;
          }
          
          proportional = P_gain * error;
          
          compensator = proportional;
          
          if( compensator > MAX_COMPENSATOR) compensator = MAX_COMPENSATOR;
          if( compensator < -MAX_COMPENSATOR) compensator = -MAX_COMPENSATOR;
          
          motor.speed(MOTOR_CRANE, compensator);
      }
// }
