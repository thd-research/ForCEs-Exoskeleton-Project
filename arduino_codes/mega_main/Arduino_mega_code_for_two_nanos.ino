/***** Arduino Mega *****/
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <DHT.h>

#include <MovingAverageFloat.h>

// for imuo values
MovingAverageFloat <16> filter0;
//for imu1 values
MovingAverageFloat <16> filter1;

//right leg 0
float p0_max = 5;
float p0_min = -0.5;

//left leg 1
float p1_max = 7.4;
float p1_min = 1.1;

int tavg = 10; 
int data0 = 1000; //arbitrary huge value for error detection
int data02 = 1000;

#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

float enc_val;
#define joyX A7
#define joyY A7

int relay = 11;

float m,k;

int a_max = 120;
int a_min = 0;

char recieved_data[100]; //Initialized variable to store recieved data

char recieved_data2[100]; //Initialized variable to store recieved data

char *temPtr; // pointer to take the data and to increment after (,)
int angle_value;
int imu1_value;

char *temPtr2; // pointer to take the data and to increment after (,)
int angle_value2;
int imu1_value2;

void setup() 
{

  odrive_serial.begin(115200);

  
  int requested_state;
int motornum = '0'-'0';
  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;

odrive_serial << "w axis" << 0 << ".controller.config.input_filter_bandwidth " << 5.0f << '\n';

odrive_serial << "w axis" << 0 << ".controller.config.input_mode " << 3 << '\n';
  
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
  Serial3.begin(9600); // Using UART channel 3 
 Serial2.begin(9600); // Using UART channel 2
  while (!Serial3);
  while (!Serial2);


}

void loop() 
{
//sensor_data();

int I = 10;
for (int i = 0; i < I; i++)
{

data0 = analogRead(A0);
filter0.add(data0);
delayMicroseconds(tavg);

}
//fetch averaged value:
data0 = filter0.get();
Serial.print(data0);Serial.print("\t");
//Serial.print(analogRead(A7));Serial.print("\t");

sensor_data2();

int  xValue = analogRead(joyX);
int  yValue = analogRead(joyY);

  Serial.flush();
//  char c = Serial.read();
//Serial.println(c);
m = (p0_max - p0_min)/1023;
k = p0_min;

float p = (m*xValue) + k;




odrive.SetPosition(0,p);                                      
delay(5);

/*
if(yValue>=0 && yValue<=249)
{
  //stay still - motor doesn't turn
 enc_val = encoder();
 odrive.SetPosition(1,enc_val);
 if(xValue<=490) //turn motor in negative direction (--)
{

enc_val = encoder();
for (float p=enc_val; p>p0_min ;p-=0.1){
  sensor_data();
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);
  if(xValue>490){
  break;
}
odrive.SetPosition(1,p);
delay(5);
}
}
}

else if(yValue>=250) //turn motor in positive direction (++)
{


enc_val = encoder();
for (float p=enc_val; p<p0_max ; p+=0.1){
  sensor_data();
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);
  if(yValue<250){
    //enc_val=encoder();
  break;
}
odrive.SetPosition(1,p);
delay(5);
}
} */




  
   }


  
float encoder()
{
  odrive_serial << "r axis" << 1 << ".encoder.pos_estimate\n";  
String enc = odrive_serial.readStringUntil('\r');
enc = enc.substring(0,6);
float enc2=enc.toFloat();



return enc2;

//string to int:
//enc.replace(".","");
//int enc2=enc.toInt();
//float enc3=float(enc2);
//enc3=enc3/1000;
}


 //left leg
void sensor_data()
{
   byte n = Serial3.available();
    if (n != 0)
  {
  byte n = Serial3.available();
    if (n != 0)
  {
    byte m = Serial3.readBytesUntil('\n', recieved_data, 200);
    recieved_data[m] = '\0';  //null-byte
    // Serial.println(myData);
    //------------------------------
    unsigned int temp_value = strtoul(recieved_data, &temPtr, 10); // Temperature sensor Value first one to send 
    
    unsigned int s2 = strtoul(temPtr + 1, &temPtr, 10); // angle sensor second one to send 
    angle_value = s2; // /100.0; // to take it here with decimal points 
    int s3 = strtoul(temPtr + 1, &temPtr, 10); // IMU value of the sensor it is the third one 
   
   imu1_value = s3; // /100.0;
  /***** if you want to Add any sensors to nano and get the values seperatly in the mega to use them 
  but add in the nano a (,) to distinquish between the values *****/
  
  /*****
  unsigned int s4 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s5 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s6 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s7 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s8 = strtoul(temPtr + 1, &temPtr, 10);
  //mapped angle value:
  
  *****/

  int mapped_angle = map(angle_value,4189,7800, 80,0);
  
    //Printing the values of the sensors 


  if(temp_value!=0){
   // Serial. print(analogRead(A7)); Serial.print("\t");
 //   Serial. print(temp_value); Serial.print("\t");
   // Serial. print(mapped_angle); Serial.print("\t");
   // Serial. print(encoder()); Serial.print("\t");
    
    //Serial. print(imu1_value+10);Serial.print("\t");
    //Serial.println();
  }
}
  }


  
}





//right leg

void sensor_data2()
{
   byte n2 = Serial2.available();
    if (n2 != 0)
  {
  byte n2 = Serial2.available();
    if (n2 != 0)
  {
    byte m2 = Serial2.readBytesUntil('\n', recieved_data2, 200);
    recieved_data2[m2] = '\0';  //null-byte
    // Serial.println(myData);
    //------------------------------
    unsigned int temp_value2 = strtoul(recieved_data2, &temPtr2, 10); // Temperature sensor Value first one to send 
    
    unsigned int s22 = strtoul(temPtr2 + 1, &temPtr2, 10); // angle sensor second one to send 
    angle_value2 = s22; // /100.0; // to take it here with decimal points 
    int s32 = strtoul(temPtr2 + 1, &temPtr2, 10); // IMU value of the sensor it is the third one 
   
   imu1_value2 = s32; // /100.0;
  /***** if you want to Add any sensors to nano and get the values seperatly in the mega to use them 
  but add in the nano a (,) to distinquish between the values *****/
int s42 = strtoul(temPtr2 + 1, &temPtr2, 10); // gz of IMU value of the sensor it is the fourth one 
int gz_value = s42;

  
  /*****
  unsigned int s4 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s5 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s6 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s7 = strtoul(temPtr + 1, &temPtr, 10);
    unsigned int s8 = strtoul(temPtr + 1, &temPtr, 10);
  //mapped angle value:
  
  *****/

  int mapped_angle2 = map(angle_value2,4490,7741, 0,80);
  


  if(temp_value2!=0){
   // Serial. print(analogRead(A7)); Serial.print("\t");
 //   Serial. print(temp_value); Serial.print("\t");
   // Serial. print(mapped_angle); Serial.print("\t");
   // Serial. print(encoder()); Serial.print("\t");
   // Serial.print(data02); Serial.print("\t");
  //  Serial.print(temp_value2);Serial.print("\t");
    Serial.print(s22);Serial.print("\t");
    Serial.print(s32);Serial.print("\t");
    Serial.println(s42);
  }
}
  }
}
