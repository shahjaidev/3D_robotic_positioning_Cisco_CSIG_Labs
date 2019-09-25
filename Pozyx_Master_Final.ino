#include <ZumoMotors.h>
#include <Pozyx_definitions.h>

#define PI 3.14159265

#include<Math.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

int user_x;
int user_y;
boolean a=true;
int grid_x, grid_y;

//SET GRID SCALE
int scale=100;

float angle_error=1.0;
boolean c=false;
float req_angle2;
boolean b=true;

float offset_degrees;
float req_angle;
sensor_raw_t sensor_raw;

euler_angles_t orientation;


uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false; // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x616f, 0x615f, 0x6167, 0x616e};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {0, 0, 3403, 4089};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {0, 4089, 4699, 0};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {1422, 1397 , 1397, 1397}; // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning


//ZumoMotors motors;

float HEADING;

//POSITIONING/COORDINATES FUNCTIONS

float get_offset()
{
//delay(10000);
float off;
float off2;
  coordinates_t P,Q;
Pozyx.doPositioning(&P, dimension, height, algorithm);


Serial.print("OFF COORDINATES");

Serial.println(P.x);

Serial.println(P.y);



orientation.heading=get_heading();

    
  while(!(orientation.heading<=1.5))
  {
 send2slave(1); 

Serial.print("O Angle is: ");
Serial.println(orientation.heading);
 orientation.heading=get_heading();
  }
 
 
 Serial.println("*** WATCH THIS");
send2slave(2); //Move Forward for 2 seconds
delay(2000);
send2slave(3); //Stop the Robot
send2slave(3); //Stop the Robot

Pozyx.doPositioning(&Q, dimension, height, algorithm);
Serial.println(Q.x);
Serial.println(Q.y);
//Find new coordinate

off=atan2((Q.y-P.y),(Q.x-P.x))*180/PI;

Serial.println(off);

off2= int((off+360))%360;

Serial.println("Offset is");
Serial.println(off2);

return  off2;
}



float get_heading()
{
Pozyx.getEulerAngles_deg(&orientation);

//Serial.println("\n Orientation Angle \n");
//Serial.println(orientation.heading);

return orientation.heading;

}

void setAnchorsManual()
{
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4)
  {
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}

void send2slave(byte c)
{

 Wire.beginTransmission(8);
// transmit to device #8

//delay(200);
  Wire.write(c);              // sends one byte
  Wire.endTransmission(); 

Serial.print(" Sent to Slave: ");
Serial.println(c);


}

void printCalibrationResult()
{
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

 
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }
}


// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor)
{
  uint16_t network_id = remote_id;
  if (network_id == NULL)
  {
    network_id = 0;
  }
  if(!use_processing)
  {
   
  grid_x=coor.x/scale;
    grid_y=coor.y/scale;
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.println(coor.z);
  }
  else
  {
    Pozyx.doPositioning(&coor, dimension, height, algorithm);
    grid_x=coor.x/scale;
    grid_y=coor.y/scale;
 
    Serial.println(" Actual COoridnates");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.println(coor.y);

    Serial.println("GRID COORDINATES (X,Y) ");
    Serial.print(grid_x);
    Serial.print(",");
    Serial.println(grid_y);
    
  
  }
}

//MOVEMENT FUNCTIONS



void forward(coordinates_t coor, sensor_raw_t sensor_raw)
{

  
Pozyx.doPositioning(&coor, dimension, height, algorithm);
  
  grid_x=coor.x/scale;
    grid_y=coor.y/scale;
Serial.println("Current Coordinates of Robot Are:");
Serial.print(" X: ");
Serial.print(grid_x);

Serial.print(" Y: ");

Serial.print(grid_y);

Serial.println("Orientation Angle of Zumo in Degrees");

orientation.heading=get_heading();

Serial.print(orientation.heading);

while(b)
{
  send2slave(2); //FORWARD
   Pozyx.doPositioning(&coor, dimension, height, algorithm);
  
  grid_x=coor.x/scale;
    grid_y=coor.y/scale;

Serial.println("X: ");
Serial.print(grid_x);

Serial.println(" Y: ");
Serial.println(grid_y);

      
      if(  ( user_x>=grid_x-1 && user_x<=grid_x+1)   && (  ( user_y>=grid_y-1) && (user_y<=grid_y+1)   ))
    {

 Serial.println("DESTINATION ARRIVED!");
    send2slave(3); //STOP
    send2slave(3);
    
Serial.println(grid_x);
Serial.println(grid_y);
b=false;
c=true;
    }
    
}


   

}


  void orient(coordinates_t coor, sensor_raw_t sensor_raw)
  {

  //To get required slope from (x1,y1) to (x2,y2)
Pozyx.doPositioning(&coor, dimension, height, algorithm);
    grid_x=coor.x/scale;
    grid_y=coor.y/scale;

 req_angle= atan2(( (user_y*scale) - coor.y),( (user_x)*scale -coor.x))*180/PI;
 //req_angle= atan2((user_y - grid_y),( user_x -grid_x))*180/PI;
 
 //Serial.println("Actual req angle: ");
 //Serial.print(req_angle);

 req_angle2=int(req_angle + offset_degrees+360)%360;
  Serial.print("Modified Required Angle in Degrees is:");
  Serial.println(req_angle2);

a=true;
  while(a)
{
  if ( ((req_angle2-angle_error)<=orientation.heading) && (orientation.heading<=(req_angle2+angle_error)))
  { 
    
    send2slave(3); //STOP
     send2slave(3); //STOP
    delay(100);
    a=false;
    forward(coor, sensor_raw);
    
  }
  
  else  {   

 Serial.println("TURNING");

   
 orientation.heading=get_heading();
  
  send2slave(1);//TURN
  Serial.println("Orientation Angle is: ");
  Serial.println(orientation.heading);
  Serial.println("Modified Required Angle in Degrees is:");
  Serial.print(req_angle2);
  a=true;
  }
    
}


}

void setup() 
{
Serial.begin(115200);
 Wire.begin();
int b;
b = Pozyx.getRawSensorData(&sensor_raw, remote_id);
 
 
 //GET OFFSET

Serial.println("Offset Degrees: ");
Serial.print(offset_degrees);
  
  //ENTER DESTINATION COORDINATES HERE (GRID COORDINATES)

  user_x=0;
  user_y=0;
  
  
  
  if(Pozyx.begin() == POZYX_FAILURE)
  {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if(!remote)
  {
    remote_id = NULL;
  }

  Serial.println(F("----------POZYX POSITIONING V1.1----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start anchor configuration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.1----------"));
  Serial.println();
  Serial.println(F("Performing manual anchor configuration:"));

  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  
  setAnchorsManual();
  
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);

  printCalibrationResult();
  delay(300);


}
  



void loop()
{

  
   offset_degrees = get_offset();
  uint8_t calibration_status = 0;
  int dt;
  int status;
 
 coordinates_t position;
  
  
  if(remote)
  {
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
    status = Pozyx.getRawSensorData(&sensor_raw, remote_id);
     status &= Pozyx.getCalibrationStatus(&calibration_status, remote_id);
  }else
  {
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }


  
  if(remote)
  {
     status = Pozyx.getRawSensorData(&sensor_raw, remote_id);
     status &= Pozyx.getCalibrationStatus(&calibration_status, remote_id);
  }
  
   else
   {
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS)
    
    {
      Pozyx.getRawSensorData(&sensor_raw);
      Pozyx.getCalibrationStatus(&calibration_status);
    }
    
    else
    {
      
      uint8_t interrupt_status = 0;
      Pozyx.getInterruptStatus(&interrupt_status);
      return;
      
    }
  }
    
    
    if (status == POZYX_SUCCESS)
  {
  
    printCoordinates(position);
  }
  else
   {//Positioning Error, we can print an error message

  }


Serial.println("ROBOT LOCATION");
printCoordinates(position);

//Serial.println("ANGLE READINGS");
 // printRawSensorData(sensor_raw);
  
  Serial.println();
    orientation.heading=get_heading();
    
orient(position, sensor_raw);


while(c)

  {
  //EMPTY LOOP TO STALL PROGRAM
  }
 
}


void printRawSensorData(sensor_raw_t sensor_raw)
{

Serial.println("\n Orientation Angle \n");
Serial.println(orientation.heading);

Serial.println("EULER ANGLES ");

  Serial.print(sensor_raw.euler_angles[0]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[1]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[2]);
  Serial.print(",");
 
}













