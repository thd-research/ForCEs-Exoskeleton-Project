/***** LSM6DSO32 *****/
#include <Adafruit_LSM6DSO32.h>
#include <Wire.h>

/*****Moving Average function*****/
#include <MovingAverageFloat.h>

/***** For IMUo values*****/
MovingAverageFloat <16> filter0;

/*****Moving average time interval in microseconds*****/
int tavg_imu = 10; 

/*****For MUX*****/
uint8_t t;
uint8_t addr;

//Testing two sensors connected to MUX: (Create a separate instance for each)
Adafruit_LSM6DSO32 dso32; //@ 0x6A
Adafruit_LSM6DSO32 dso32_2; //@ 0x6A

#define TCAADDR 0x70

void tcaselect (uint8_t i)
{
if (i > 7) return;
Wire.beginTransmission(TCAADDR);
Wire.write(1 << i);
Wire.endTransmission();
}

//gyro values:
float imu0_gz;
float imu1_gz;

/***** TEMPERATURE SENSOR*****/

#include <DHT.h>
#define DHTPIN 9     // pin we're connected to
#define DHTTYPE DHT22   // DHT 22 
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino


int temperature; //Temperature  value

/*****ANGLE SENSOR*****/

#include <AS5048A.h>
float angle_value;
AS5048A angleSensor(10, true);
char values_of_angle[10];


char values_of_imu[10];

void setup() 
{
Serial.begin(9600);
Wire.begin();
delay(100);

/*      INITIALIZE FIRST SENSOR   */
t = 0;
tcaselect(t);
dso32.begin_I2C();
//Port:
//Serial.print("TCA Port: ");Serial.println(t);

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
delay(100);

    dht.begin(); //Temperature sensor intilliaze 
    angleSensor.begin(); //Angle sensor intilliaze
}




void loop() 
{
     /*****TEMPERATURE SENSOR*****/
   
    temperature= dht.readTemperature(); // INTEGEAR VALUE
    //dtostrf(temperature,3,2,temperature_value); //(float, bytes,number of number after decimal,char variable)  //String variables
    //Serial.write(temperature_value,6);
  
      /*****ANGLE SENSOR*****/
    
    angle_value = angleSensor.getRotationInDegrees();
    angle_value = angle_value *100; 
    dtostrf(angle_value,3,0,values_of_angle); //(float, bytes,number of number after decimal,char variable)  //CHARACTER VARIABLE
    //Serial.write(values_of_angle,6);

float imu0_val = imu0();

dtostrf(imu0_val,3,0,values_of_imu);
  /***** Printing the Values *****/
   Serial.print(temperature);  Serial.print(','); Serial.print(values_of_angle); Serial.print(','); Serial.print(values_of_imu);  Serial.print('\n');
   /*Serial.print(','); Serial.println(imu0_val); Serial.print('\n');*/
   delay(50);
}




//FUNCTION 1: To read averaged IMU value from IMU Sensor connected to channel 0:
float imu0()
{
t = 0;
 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

   tcaselect(t);

float data0;
float data1;

int I = 10; //number of moving average points to be taken
//float tavg_torque = 1000; //delay between each reading in Microseconds

for (int i = 0; i < I; i++)
{
data0 = gyro.gyro.z*100;
filter0.add(data0);
delayMicroseconds(tavg_imu);
}
//fetch averaged value:
data0 = filter0.get();

//ignore for now:
 // data0 = (data0*100)+ 140;

  return data0;
}
