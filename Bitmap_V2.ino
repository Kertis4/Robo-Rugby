#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>

LCD16x2 lcd;

int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;

int button_state = lcd.readButtons();
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

  Wire.begin();


pinMode(E1, OUTPUT); //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
pinMode(E2, OUTPUT);
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);


  lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite(button_state);
  
  //button_state = lcd.readButtons();
  //bool button_1 = ~button_state & 0b0001;
 // bool button_2 = ~button_state & 0b0010;
  //bool button_3 = ~button_state & 0b0100;



  
}

void loop(){ //this code loops continually after running through 'setup' once
 lcd.readButtons();
 bool button_1 = (button_state & 0b0001) !=0;
 bool button_2 = (button_state & 0b0010) !=0;
 bool button_3 = (button_state & 0b0100) !=0;

 Serial.println(button_state);
lcd.lcdClear();
lcd.lcdGoToXY(1,1);
lcd.lcdWrite("W");
if (button_1 == 1)
 { 
Serial.println("button 1 pressed");
 //analogWrite (E1,254); //Set M1 speed
 //digitalWrite(M1,HIGH); //Set M1 direction 
 
 //analogWrite (E2,254); //Set M2 speed 
 //digitalWrite(M2,LOW) ;//Set M2 speed
 //delay(200);
 }  
 if (button_2 == 1)
 { 
Serial.println("button 2 pressed");
 //analogWrite (E1,50); //Set M1 speed
 //digitalWrite(M1,HIGH); //Set M1 direction 
 
 //analogWrite (E2,254); //Set M2 speed 
 //digitalWrite(M2,LOW) ;//Set M2 speed
 //delay(200);
}
if (button_3 == 1)
 { 
Serial.println("button 3 pressed");
 //analogWrite (E1,254); //Set M1 speed
 //digitalWrite(M1,HIGH); //Set M1 direction 
 
 //analogWrite (E2,50); //Set M2 speed 
 //digitalWrite(M2,LOW) ;//Set M2 speed
 //delay(200);
}}
