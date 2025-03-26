#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>

LCD16x2 lcd;


int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;


int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control

void GoForward(int speed)
{
  analogWrite (E1,speed); //Set M1 speed digitalWrite(M1,HIGH); //Set M1 direction 
  return; //end
}


void setup(){ //this code executes once at start
int b;

  Wire.begin();


pinMode(E1, OUTPUT); //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
pinMode(E2, OUTPUT);
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);

pinMode(A0, INPUT_PULLUP);

  lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Voltage");

  b = lcd.readButtons(); //Sample the state of the large white buttons

while(b == 15) // A value of 15 will only be seen if no buttons are pressed
{
  b = lcd.readButtons(); //Sample the state of the large white buttons
}

// Will only proceed as far as here once a button is pressed

// We want execution to get "trapped" in the above loop: otherwise, the programme would turn on the motors straigh away, and your robot might go sailing off the bench!
  
}

  bool sensor_state = 0;
  bool sensor_state_old = 0;
  int counter = 0;
  float circumference = 3.1416*30;
  int distance;
  
void loop(){ //this code loops continually after running through 'setup' once


  int mapped_voltage = map(analogRead(A0), 0, 1023, 0, 100);
 

  if (mapped_voltage>30)
    {
      sensor_state = 0;
    }
  else
    {
      sensor_state = 1;
    }
  if (sensor_state>sensor_state_old)
    {
     counter++;
    }

    sensor_state_old = sensor_state;

    distance = counter*circumference;
     
    lcd.lcdClear();
    lcd.lcdGoToXY(1,1);
    distance = (distance*100.0) / 100.0;
    lcd.lcdWrite(distance); 

   
}
