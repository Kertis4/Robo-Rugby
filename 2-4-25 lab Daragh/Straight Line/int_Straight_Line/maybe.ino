#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>
#include <Arduino_NineAxesMotion.h>
#include <EnableInterrupt.h>
LCD16x2 lcd;
NineAxesMotion mySensor;

volatile int CycleCount = 0; // use volatile for shared variables
volatile int MatchLength = 60; // 60 seconds is 1 minutes

int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control
int mode;

void turninplace(int target)
{
  int heading = mySensor.readEulerHeading();
  int heading_error = heading - target;
  while(abs(heading_error) > 3)
  {
    heading = mySensor.readEulerHeading();
    heading_error = heading - target;

    if(heading_error <= -180)
    {
        heading_error += 360;
    }
    if(heading_error > 180)
    {
      heading_error -= 360;
    }

    int gain1 = 10;     //how agressively the robot steers when it is far from the target heading
    int gain2 = 15;     //how agressively the robot steers when it is close to the target heading

    int steering_effort = gain1*pow(heading_error,3)*0.001 + gain2*heading_error*0.1;
    steering_effort = constrain(steering_effort, -255, 255);

    int right_power = steering_effort;
    int left_power = steering_effort*-1;

      if(left_power >= 0)
      {
          analogWrite (E1,left_power); //Set M1 speed
          digitalWrite(M1, HIGH); //Set M1 direction
      }
      if(left_power < 0)
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
      if(right_power < 0)
      {
          right_power *= -1;
          analogWrite (E2,right_power); //Set M2 speed
          digitalWrite(M2,LOW);//Set M2 speed
      }
  }
  return;
}

void setup()
{ //this code executes once at start
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
    
    if(b == 14)
    {
      mode = 1;
      lcd.lcdGoToXY(1,1); //Move to the right place for printing this sample
      lcd.lcdWrite("                 "); //Clear it our with four spaces
      lcd.lcdWrite("Rock");
    }

    if(b == 13)
    {
      mode = 2;
      lcd.lcdGoToXY(1,1); //Move to the right place for printing this sample
      lcd.lcdWrite("                 "); //Clear it our with four spaces
      lcd.lcdWrite("Paper"); 
    }

    if(b == 11)
    {
        mode = 3;
        lcd.lcdGoToXY(1,1); //Move to the right place for printing this sample
        lcd.lcdWrite("                 "); //Clear it our with four spaces
        lcd.lcdWrite("Scissors"); ;
    }


arm_and_start();
}

void start_lcd16()
{
    //Wire.begin();
    lcd.lcdClear();
    lcd.lcdGoToXY(1,1); //Write in the static text labels for later
    lcd.lcdWrite("H");
   
    lcd.lcdGoToXY(6,1);
    lcd.lcdWrite("E");
    
    lcd.lcdGoToXY(12,1);
    lcd.lcdWrite("T");

    lcd.lcdGoToXY(1,2); //Write in the static text labels for later
    lcd.lcdWrite("L");
   
    lcd.lcdGoToXY(6,2);
    lcd.lcdWrite("R");
    
  if(mode == 1)
  {
    lcd.lcdGoToXY(12,2);
    lcd.lcdWrite("Rock");
  }
  if(mode == 2)
  {
    lcd.lcdGoToXY(12,2);
    lcd.lcdWrite("Paper");
  }
  if(mode == 3)
  {
    lcd.lcdGoToXY(12,2);
    lcd.lcdWrite("Sciss");
  }

}

void loop() //this code loops continually after running through 'setup' once
{
//Set LCD Screen
//========================================================================
  static bool startlcd = 1;
  unsigned int start_time;
  if(startlcd == 1)
  {
    start_lcd16();
    start_time = millis();  
    startlcd = 0;
  }
//==========================================================================
//Sensors
//=============================================================================================================
  int heading = mySensor.readEulerHeading(); //read all the data we want from the sensor into our variables
  unsigned int read_9axis_time = millis();
  int run_time = read_9axis_time - start_time;
//=============================================================================================================
//Robot Directions
//==============================================================================================================
  int target_heading;
  if(mode == 1)
  {
    target_heading = 0;
  }
  else if(mode == 2)
  {
    if(run_time < 1000)
    {
      target_heading = 0;
    }
    else if(run_time>=250 && run_time<3000)
    {
      target_heading = 20;
    }
    //else if(run_time>=2000 && run_time<3000)
    //{
      //target_heading = 20;
    //}
    else if(run_time>=3000 && run_time<4000)
    {
      target_heading = 10;
    }
    else if(run_time >= 4000)
    {
      target_heading = 0;
    }
  }
  else if(mode == 3)
  {
    if(run_time < 1000)
    {
      target_heading = 0;
    }
    else if(run_time>=1000 && run_time<3000)
    {
      target_heading = 330;
      /*static bool turn1 = 0;
      if(turn1 == 0)
      {
        turninplace(target_heading);
        turn1 = 1;  
      }*/
    }
    else if(run_time>=3000 && run_time<4000)
    {
      target_heading = 300;
      /*static bool turn2 = 0;
      if(turn2 == 0)
      {
        turninplace(target_heading);
        turn2 = 1;  
      }*/
    }
    else if(run_time >= 4000)
    {
      target_heading = 246;
      /*static bool turn3 = 0;
      if(turn3 == 0)
      {
        turninplace(target_heading);
        turn3 = 1;  
      }*/
    }
  }

  int heading_error = heading - target_heading;

  if(heading_error < -180)
  {
      heading_error += 360;
  }
  else if(heading_error > 180)
  {
    heading_error -= 360;
  }
//=============================================================================================================================
//Motor Control
//================================================================================================
  int gain1 = 10;     //how agressively the robot steers
  
  int steering_effort = gain1*heading_error;

  if(heading_error < 0)
  {
      steering_effort -= 1;
  }
  if(heading_error > 0)
  {
      steering_effort += 1;
  }

  int left_trim = 250;  //change these values so the robot drives straight with no correction
  int right_trim = 255;

  int right_power = right_trim + steering_effort;
  int left_power = left_trim - steering_effort;

  left_power = constrain(left_power, -left_trim, left_trim);
  right_power = constrain(right_power, -right_trim, right_trim);

  /*if(left_power<=0 || right_power<=0)
  {
    turninplace(target_heading);
  }*/

  analogWrite (E1,left_power); //Set M1 speed
  digitalWrite(M1,HIGH); //Set M1 direction 
 
  analogWrite (E2,right_power); //Set M2 speed 
  digitalWrite(M2,HIGH) ;//Set M2 speed

//=============================================================================================================================
//16x2 LCD Display
//==================================================================================================================================
  int refresh_time = millis();
    while(refresh_time - read_9axis_time < 25)
    {
        refresh_time = millis(); //little delay so we don't confuse the I2C bus FIXME: is there a more elegant way to do this?
    }
    delay(25);
    lcd.lcdGoToXY(2,1); //Move to the right place for printing this sample
    lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(heading);

    lcd.lcdGoToXY(7,1); //Move to the right place for printing this sample
    lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(heading_error);

    lcd.lcdGoToXY(13,1);
    lcd.lcdWrite("   ");
    lcd.lcdWrite((float)run_time*0.001,2);

    lcd.lcdGoToXY(2,2); //Move to the right place for printing this sample
    lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(left_power);
    
    lcd.lcdGoToXY(7,2); //Move to the right place for printing this sample
    lcd.lcdWrite("    "); //Clear it our with four spaces
    lcd.lcdWrite(right_power);
//============================================================================================================================
}
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
