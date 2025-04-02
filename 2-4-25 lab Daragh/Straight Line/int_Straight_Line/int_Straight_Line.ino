// Example 9 axis sensor code for UCD EEEN10020
// By Dr. Paul Cuffe paul.cuffe@ucd.ie

#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>
#include <Arduino_NineAxesMotion.h>

LCD16x2 lcd;
NineAxesMotion mySensor;

int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control


void setup() //This code is executed once
{
  int buttons;
  float positions[8][2];


  //Peripheral Initialization
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  Wire.begin();

  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor

  pinMode(E1,OUTPUT); //Set up motor pins
  pinMode(E2,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);

  pinMode(2,OUTPUT);

  digitalWrite(2,LOW) ;

  
  lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Press white key");

  buttons = lcd.readButtons(); //Sample the state of the large white buttons

while(buttons == 15) // A value of 15 will only be seen if no buttons are pressed
{
  buttons = lcd.readButtons(); //Sample the state of the large white buttons
}

//lcd.lcdClear();
//lcd.lcdGoToXY(12,1);
//lcd.lcdWrite("R");
//lcd.lcdGoToXY(12,2);
//lcd.lcdWrite("Z");

delay(50);
 digitalWrite(2,LOW) ;//Set M2 speed
}



void loop()
{
  //Sensor Data
//===================================================
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
}


    

    

    
