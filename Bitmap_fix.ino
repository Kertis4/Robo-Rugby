#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>

LCD16x2 lcd;


int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;


int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control


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
 // lcd.lcdWrite("Voltage");

  b = lcd.readButtons(); //Sample the state of the large white buttons

while(b == 15) // A value of 15 will only be seen if no buttons are pressed
{
  b = lcd.readButtons(); //Sample the state of the large white buttons
}





  
void loop(){ //this code loops continually after running through 'setup' once

if (b==1)
{Serial.println("button 1 pressed");
}
if (b==2)
{Serial.println("button 1 pressed");
}
if (b==4)
{Serial.println("button 1 pressed");
}
if (b==7)
{Serial.println("button 1 pressed");
}

 
   
}
