#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>
#include <Arduino_NineAxesMotion.h>
#include <EnableInterrupt.h>
LCD16x2 lcd;
NineAxesMotion mySensor;

int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;

volatile int CycleCount = 0; // use volatile for shared variables
volatile int MatchLength = 60; // 60 seconds is 1 minutes


int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control
int mode;


void setup(){ //this code executes once at start
int b;
  I2C.begin(); 
  Wire.begin();

 //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor

pinMode(E1, OUTPUT); //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
pinMode(E2, OUTPUT);
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);

pinMode(2,OUTPUT);

digitalWrite(2,LOW) ;
lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Rock, Paper, Scissors");

  b = lcd.readButtons(); //Sample the state of the large white buttons

while(b == 15) // A value of 15 will only be seen if no buttons are pressed
{
  b = lcd.readButtons(); //Sample the state of the large white buttons
}
arm_and_start();


}



void loop(){ //this code loops continually after running through 'setup' once
int b = lcd.readButtons();
int a = b;

  /*lcd.lcdGoToXY(3,1); //Move to the right place for printing this sample
 lcd.lcdWrite("                 "); //Clear it our with four spaces
 lcd.lcdWrite(b); */

if (b==14)
{mode = 1
  
 lcd.lcdGoToXY(1,1); //Move to the right place for printing this sample
 lcd.lcdWrite("                 "); //Clear it our with four spaces
 lcd.lcdWrite("Rock");
  
}
if (b==13)
{mode = 2;
 lcd.lcdGoToXY(3,1); //Move to the right place for printing this sample
 lcd.lcdWrite("                 "); //Clear it our with four spaces
 lcd.lcdWrite("Paper"); 
}
if (b==11)
{mode = 3;
  
 lcd.lcdGoToXY(3,1); //Move to the right place for printing this sample
 lcd.lcdWrite("                 "); //Clear it our with four spaces
 lcd.lcdWrite("Scissors"); ;
}

while(a==b)
{
  b = lcd.readButtons();
}
//==========================================================
// ROCK
//==========================================================
while(mode ==1)
{
  
}
//==========================================================
// SCISSORS
//==========================================================
while(mode ==3)
{
float raw_heading = mySensor.readEulerHeading(); //read all the data we want from the sensor into our variables
    int read_9axis_time = millis();

//==========================================================

//==========================================================

//Data Conversion
//==========================================================
  int heading = raw_heading*100;
  int target_heading = 0.00*100;
//========================================================

//Direction Finding
//==========================================================

      while(target_heading<0)
      {
        target_heading+=36000;
      }
      while(target_heading>36000)
      {
        target_heading-=36000;
      }

    int heading_error = heading - target_heading;

      if(heading_error<-18000)
      {
        heading_error +=36000;
      }
      else if(heading_error>18000)
      {
        heading_error -=36000;
      }

//Motor Control
//==========================================================
  int gain1 = 20;
  int gain2 = 30;

  int left_trim = 255;
  int right_trim = 255;

    int steering_effort = (gain1*heading_error*heading_error*heading_error*0.0000001 + gain2*heading_error*0.1)*0.01;

    int right_power = right_trim + steering_effort;
    int left_power = left_trim - steering_effort;

      left_power = constrain(left_power, -left_trim, left_trim);
      right_power = constrain(right_power, -right_trim, right_trim);

    
      if(left_power >= 0)
      {
          analogWrite (E1,left_power); //Set M1 speed
          digitalWrite(M1, HIGH); //Set M1 direction
      }
      else
      {
          left_power *= -1;
          analogWrite (E1,left_power); //Set M1 speed
          digitalWrite(M1, LOW); //Set M1 direction
      }

      if(right_power >= 0)
      {
          analogWrite (E2,right_power); //Set M2 speed
          digitalWrite(M2,HIGH);//Set M2 speed
      }
      else
      {
          right_power *= -1;
          analogWrite (E2,right_power); //Set M2 speed
          digitalWrite(M2,LOW);//Set M2 speed
      }
//==========================================================

//LCD Screen Control
//==========================================================
int refresh_time = millis();
float error = heading_error*0.01;

while(refresh_time - read_9axis_time < 25)
{
    refresh_time = millis(); //little delay so we don't confuse the I2C bus FIXME: is there a more elegant way to do this?
}
  lcd.lcdClear();

      lcd.lcdGoToXY(1,1); //Write in the static text labels for later
    lcd.lcdWrite("H");
      lcd.lcdGoToXY(2,1); //Move to the right place for printing this sample
      lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(raw_heading, 1); //Want one decimal place
    
      lcd.lcdGoToXY(6,1); //Write in the static text labels for later
    lcd.lcdWrite("E");
      lcd.lcdGoToXY(7,1); //Move to the right place for printing this sample    lcd.lcdWrite("    "); //Clear it our with four spaces
      lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(error, 1); 

      lcd.lcdGoToXY(1,2); //Write in the static text labels for later
    lcd.lcdWrite("L");
      lcd.lcdGoToXY(2,2); //Move to the right place for printing this sample
      lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(left_power); //Want one decimal place
    
      lcd.lcdGoToXY(6,2); //Write in the static text labels for later
    lcd.lcdWrite("R");
      lcd.lcdGoToXY(7,2); //Move to the right place for printing this sample
      lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(right_power); //Want one decimal place
//==========================================================


}}
//=====================================================================
//=====================================================================
// END OF LOOP + START OF SHUT DOWN
//=====================================================================
//=====================================================================
void arm_and_start() {

int buttons = 0;

float timeleft;

long countdown_int;

long fudge = 0;
bool mode = 0;
long timeflipped = 0;

  Wire.begin();
   
  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);

  lcd.lcdGoToXY(1,2);
  lcd.lcdWrite("v");

  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("SYNC ");

  lcd.lcdGoToXY(12,1);
  lcd.lcdWrite(" SYNC"); 

  lcd.lcdGoToXY(4,2);
  lcd.lcdWrite("[--set--]");

  
  buttons = lcd.readButtons(); //Sample the state of the large white buttons

do
{
  buttons = lcd.readButtons(); //Sample the state of the large white buttons


  if (buttons == 14) //leftmost button depressed
  {
  fudge = fudge + 450;
  }

  buttons = lcd.readButtons(); //Sample the state of the large white buttons

  if (buttons == 9 && (millis() - timeflipped) > 250) //both middle buttons depressed
  {
  mode = !mode;
  timeflipped = millis();

        if(!mode) //syncing safe mode
        {
          lcd.lcdSetBlacklight(100);
        
         lcd.lcdGoToXY(1,1);
         lcd.lcdWrite("SYNC ");
        
          lcd.lcdGoToXY(12,1);
          lcd.lcdWrite(" SYNC"); 
        
          lcd.lcdGoToXY(4,2);
          lcd.lcdWrite("[--set--]"); 
          
        }
        else if(mode) //armed mode
        {
          lcd.lcdSetBlacklight(400);
          lcd.lcdGoToXY(1,1);
          lcd.lcdWrite("ARMED");
        
         lcd.lcdGoToXY(12,1);
         lcd.lcdWrite("ARMED"); 
        
          lcd.lcdGoToXY(4,2);
          lcd.lcdWrite("[-disarm-]"); 
        }
          
  }


countdown_int = (millis() + fudge) % 10000;

countdown_int = 9998 - countdown_int;

  lcd.lcdGoToXY(7,1);
  lcd.lcdWrite((float) countdown_int/1000, 2);
  
}
while(!(mode == 1 && countdown_int <= 750)); 

  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);
  lcd.lcdGoToXY(6,2);
  lcd.lcdWrite("Begin!");

//Now set up an interupt to trigger after the appropriate number of minutes to shut down the robot
  
 enableInterrupt(11, CheckCycle, RISING); //Attach an interrupt to pin 11
 tone(11, 31); //Output a 31 Hz square wave to pin 11 to trigger this interrupt
}

void CheckCycle(void)
{
    CycleCount = CycleCount + 1;  //Check how many times we've been here
    
  if(CycleCount == 31 * MatchLength) 
  {
  //turn off motors
   analogWrite (E1,0); //Turn off M1
   analogWrite (E2,0); //Turn off M2
      
      while(1) //loop forever to shut down Arduino
      {
      }
  }
}
