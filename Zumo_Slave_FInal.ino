#include <Wire.h>
#include <ZumoMotors.h>

ZumoMotors motors;
//byte c;

//char c;

void setup() 
{
Serial.begin(9600);  
Serial.println(" Slave starts");



  motors.setSpeeds(0, 0);
  Wire.begin(8);                // join i2c bus with address #8
Wire.onReceive(receiveEvent); // register event
     
}

  void loop() 
{
  
  /*
if(Serial.available()>0)
{
c=Serial.read();
Serial.println(c); 
}
*/

//Serial.print(c);
 
  delay(100);
}

  void receiveEvent(int howMany) 
{
  while (1 < Wire.available()) 
  { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);   
    
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         
   switch (x)
  {

  case 1: motors.setSpeeds(0, 108); //Turn till told not to
  Serial.println("TURNING");
  break;

  case 2: motors.setSpeeds(160,160); break; //FORWARD

  case 3: motors.setSpeeds(0,0); break; //STOP
  
  }      

  
}


