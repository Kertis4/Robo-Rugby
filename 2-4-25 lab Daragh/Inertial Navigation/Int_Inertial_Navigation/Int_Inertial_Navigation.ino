// Example 9 axis sensor code for UCD EEEN10020
// By Dr. Paul Cuffe paul.cuffe@ucd.ie

#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>
//#include <NineAxesMotion.h>
#include <Arduino_NineAxesMotion.h>
#include <EnableInterrupt.h>

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


  lcd.lcdClear();

  lcd.lcdGoToXY(1,1); //Write in the static text labels for later
  lcd.lcdWrite("H");
   
  lcd.lcdGoToXY(6,1);
  lcd.lcdWrite("P");
    
  lcd.lcdGoToXY(12,1);
  lcd.lcdWrite("R");

  lcd.lcdGoToXY(1,2); //Write in the static text labels for later
  lcd.lcdWrite("j");
   
  lcd.lcdGoToXY(6,2);
  lcd.lcdWrite("i");
    
  lcd.lcdGoToXY(12,2);
  lcd.lcdWrite("k");

  delay(50);

  digitalWrite(2,LOW) ;//Set M2 speed
}

static float time_new = millis();

void loop()
{
//Sensor Data
//===============================================================================================
  //read all the data we want from the sensor into our variables
    int heading = mySensor.readEulerHeading()*100;                              //takes the current compass heading
    float roll = mySensor.readEulerRoll();
    float pitch = mySensor.readEulerPitch();

    int Aj = mySensor.readLinearAccelX()*100;
    int Ai = mySensor.readLinearAccelY()*100;
    int Ak = mySensor.readLinearAccelZ()*100;
//===============================================================================================

//Sensor Correction
//===============================================================================================
    pitch *= -1;
    Aj *= -1;
//===============================================================================================

// Inertial Navigation
//===============================================================================================
    static int dist = 0;
    static int veloc_max = 0;
    static int x_accel_init = 0;
    static int x_veloc = 0;
    static int x_pos = 0;
    static int y_accel_init = 0;
    static int y_veloc = 0;
    static int y_pos = 0;

    int dtime, time_init, compass, accel_mag, accel_angle, x_accel, dx_accel, dx_veloc, dx_pos, y_accel, dy_accel, dy_veloc, dy_pos;

    time_init = time_new;                                        //resets time initial value
    time_new = millis();                                         //takes the time of the arduino internal clock
    compass = heading*1.745;                                     //turns heading to radians
    accel_mag = 10*sqrt(Aj*Aj + Ai*Ai);                          //takes the current acceleration magnitude
    accel_angle = atan((float)Ai/(float)Aj);                     //how much the acceleration vector heading differs from the robot heading

    if(accel_mag == 0)
    {
        accel_angle = 0;
    }
    if(Aj<0)
    {
        if(Ai>=0)
        {
            accel_angle = accel_angle+31416;                      //fourth quadrant
        }
        if(Ai<0)
        {
            accel_angle = accel_angle-31416;                      //third quadrant
        }
    }

    dtime = time_new - time_init;                                 //time delta since the last time the loop ran

    x_accel = accel_mag*sin((compass + accel_angle)*0.0001);      //acceleration in the X axis, or East/West direction if 0 degrees is North
    dx_accel = x_accel - x_accel_init;                            //acceleration delta since the last time the loop ran
    x_accel_init = x_accel;                                       //resets acceleration initial value
  
    dx_veloc = dtime*(x_accel-0.5*dx_accel)*0.001;                //velocity delta since the last time the loop ran
     x_veloc+= dx_veloc;                                          //keeps track of velocity using the trapezoidal rule
    
    dx_pos = dtime*(x_veloc-0.5*dx_veloc)*0.001;                  //position delta since the last time the loop ran
    x_pos += dx_pos;                                              //keeps track of position using trapezoidal rule


    y_accel = accel_mag*cos((compass + accel_angle)*0.0001);      //acceleration in the Y axis, or North/South direction if 0 degrees is North
    dy_accel = y_accel - y_accel_init;                            //acceleration delta since the last time the loop ran
    y_accel_init = y_accel;                                       //resets acceleration initial value
 
    dy_veloc = dtime*(y_accel-0.5*dy_accel)*0.001;                //velocity delta since the last time the loop ran
    y_veloc += dy_veloc;                                          //keeps track of velocity using the trapezoidal rule

    dy_pos = dtime*(y_veloc-0.5*dy_veloc)*0.001;                  //position delta since the last time the loop ran
    y_pos += dy_pos;                                              //keeps track of position using trapezoidal rule
    
    int veloc_mag = sqrt(x_veloc*x_veloc + y_veloc*y_veloc);
    veloc_max = max(veloc_max,veloc_mag);
    dist += sqrt(dx_pos*dx_pos + dy_pos*dy_pos);
    int accel_max;
    accel_max = max(accel_max, accel_mag);

//===============================================================================================

//Waypoints
//===============================================================================================
    static int Stage = 1;
    //bool reverse;
    
    if(Stage == 5)
    {
      Stage = 1;
    }

    float x_tar, y_tar;

    if(Stage == 1)
    {
        x_tar = 0;                          //Temporary Placeholder
        y_tar = 1000;                       //Temporary Placeholder
        //reverse = 0;
    }
    if(Stage == 2)
    {
        x_tar = 1000;                       //Temporary Placeholder
        y_tar = 1000;                       //Temporary Placeholder
        //reverse = 0;
    }
    if(Stage == 3)
    {
        x_tar = 1000;                       //Temporary Placeholder
        y_tar = 0;                          //Temporary Placeholder
        //reverse = 0;
    }
    if(Stage == 4)
    {
        x_tar = 0;                          //Temporary Placeholder
        y_tar = 0;                          //Temporary Placeholder
        //reverse = 0;
    }
//===============================================================================================

//Direction Finding
//===============================================================================================
    //bool stop;
    
    //float time_since_stage;
    int x_resultant = x_tar-x_pos;
    int y_resultant = y_tar-y_pos;
    int target_heading = atan((float)x_resultant/(float)y_resultant)*5730;
      if(y_tar < y_pos)
      {
          target_heading += 18000;
      }
      while(target_heading < 0)
      {
          target_heading += 36000;
      }
      while(target_heading > 36000)
      {
          target_heading -= 36000;
      }

    /*if(reverse == 1)
    {
        heading += 180; 
        if(heading>360)
        {
          heading -=360;
        }
    }*/

    int heading_error = heading - target_heading;

      if(heading_error < -18000)
      {
          heading_error += 36000;
      }
      if(heading_error > 18000)
      {
          heading_error -= 36000;
      }

      int distance = sqrt(x_resultant*x_resultant + y_resultant*y_resultant);

      if(distance < 50)
      {
          static int closest_point = 50;
          closest_point = min(distance, closest_point);

          if(distance > closest_point)
          {
              //time_since_stage = millis()/1000.0;
              Stage ++;
          }
      }
//===============================================================================================

//Motor Control
//===============================================================================================
    /*if(time_new - time_since_stage < 0.5)
    {
        stop = 1;
    }
    else
    {
        stop = 0;
    }*/

    int gain1 = 20;
    int gain2 = 30;

    int left_trim = 255;
    int right_trim = 255;

    int steering_effort = (gain1*pow(heading_error,3)*0.000001 + gain2*heading_error)*0.001;

    int right_power = right_trim + steering_effort;
    int left_power = left_trim - steering_effort;

    left_power = constrain(left_power, -left_trim, left_trim);
    right_power = constrain(right_power, -right_trim, right_trim);

    /*if(stop == 0)
    {
        if(reverse == 0)
        {*/
            if(right_power >= 0)
            {
                analogWrite (E1, right_power);
                digitalWrite(M1, HIGH);
            }
            else//if(right_power < 0)
            {
                right_power *= -1;
                analogWrite (E1,right_power); //Set M1 speed
                digitalWrite(M1, LOW); //Set M1 direction
            }
            if(left_power >= 0)
            {
               
                analogWrite (E2,left_power); //Set M2 speed
                digitalWrite(M2, HIGH);//Set M2 speed
            }
            else//if(left_power < 0)
            {
                left_power *=-1;
                analogWrite (E2,left_power); //Set M2 speed
                digitalWrite(M2, LOW);//Set M2 speed            
            }
        /*}
        if(reverse == 1)
        {
        if(right_power < 0)
            {
                right_power *= -1;
                analogWrite (E1, right_power); //Set M1 speed
                digitalWrite(M1, HIGH); //Set M1 direction
            }
            if(right_power >= 0)
            {
                analogWrite (E1,right_power); //Set M1 speed
                digitalWrite(M1, LOW); //Set M1 direction
            }
            if(left_power < 0)
            {
                left_power *=-1;
                analogWrite (E2,left_power); //Set M2 speed
                digitalWrite(M2, HIGH);//Set M2 speed
            }
            if(left_power >= 0)
            {
                analogWrite (E2,left_power); //Set M2 speed
                digitalWrite(M2, LOW);//Set M2 speed            
            }
        }
    }*/

   /* if(stop == 1)
    {
    analogWrite (E1,0); //Set M1 speed
    analogWrite (E2,0); //Set M2 speed
    digitalWrite(M1, HIGH); //Set M1 direction
    digitalWrite(M2, HIGH);//Set M2 speed
    }*/

//===============================================================================================

//Serial Monitor
//===============================================================================================
    //Euler
    Serial.print(" H: ");
    Serial.print(heading);
    Serial.print(", ");

    Serial.print(" R: ");
    Serial.print(roll);
    Serial.print(",");

    Serial.print(" P: ");
    Serial.print(pitch);
    Serial.print(", ");

    //Acceleration

    Serial.print(" Accel_j: ");
    Serial.print(Aj);
    Serial.print(", ");

    Serial.print(" Accel_i: ");
    Serial.print(Ai);
    Serial.print(", ");

    Serial.print(" Accel_k: ");
    Serial.print(Ak);
    Serial.print(", ");

    Serial.print(" VelocX: ");
    Serial.print(x_veloc);
    Serial.print(", ");
    
    Serial.print(" VelocY: ");
    Serial.print(y_veloc);
    Serial.print(", ");
    
    Serial.print(" PosX: ");
    Serial.print(x_pos);
    Serial.print(", ");
    
    Serial.print(" PosY: ");
    Serial.print(y_pos);
    Serial.print(", ");

    Serial.print(" Total Distance: ");
    Serial.print(dist);
    Serial.print(", ");

    Serial.print(" Refresh Rate: ");
    Serial.print(dtime);
    Serial.print(", ");

    Serial.println(); //end of serial printing line
//===============================================================================================

//LCD Screen Control
//===============================================================================================
    float time_refresh = millis();

    while(time_refresh-time_new < 25)
    {
        time_refresh = millis();
    }
    
  lcd.lcdClear();
 
        lcd.lcdGoToXY(2,1); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(heading,1); //Want one decimal place

        lcd.lcdGoToXY(7,1); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(pitch,1); 

        lcd.lcdGoToXY(13,1); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(roll,1); 
    
        lcd.lcdGoToXY(2,2); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(Aj,1); //Want one decimal place

        lcd.lcdGoToXY(7,2); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(Ai,1); 

        lcd.lcdGoToXY(13,2); //Move to the right place for printing this sample
        lcd.lcdWrite("    "); //Clear it our with four spaces
        lcd.lcdWrite(Ak,1); 
    
//===============================================================================================
}
