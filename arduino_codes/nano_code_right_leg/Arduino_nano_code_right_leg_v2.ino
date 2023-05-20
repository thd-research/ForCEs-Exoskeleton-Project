
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

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
//uint8_t t;
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
#define DHTPIN 7     // pin we're connected to
#define DHTTYPE DHT22   // DHT 22 
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino

int temperature; //Temperature  value

/*****ANGLE SENSOR*****/
#include <AS5048A.h>
int angle_value;
AS5048A angleSensor(10, true);
char values_of_angle[10];

char values_of_imu[10];

void setup() 
{
Serial.begin(9600);

Wire.begin();

delay(100);

dht.begin(); //Temperature sensor intilliaze 
angleSensor.begin(); //Angle sensor intilliaze

/* Initialise the bn055 sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }




}



void loop() 
{
temperature= dht.readTemperature(); // INTEGEAR VALUE

angle_value = angleSensor.getRawRotation();

//float imu0_val = imu0();


  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);


imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

float gx,gy,gz;
float ax,ay,az;

 // gz = printEvent2(&angVelocityData);

gz = printEvent2(&accelerometerData);
  ax = printEvent(&accelerometerData);


int axx = ax*100;
int gzz = gz*100;

Serial.print(temperature);Serial.print(",");
Serial.print(angle_value);Serial.print(",");
Serial.print(axx);Serial.print(",");
Serial.println(gzz);


      delay(BNO055_SAMPLERATE_DELAY_MS);

}



float printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //Serial.print("Accl:");
    //x = event->acceleration.x;
    x = event->acceleration.x;
  //  y = event->acceleration.y;
  //  z = event->acceleration.z;
  }




  else {
    Serial.print("Unk:");
  }

  //Serial.print("\tx= ");
 // Serial.print(x);
return x;
}







float printEvent2(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem



  if (event->type == SENSOR_TYPE_GYROSCOPE) {
   // Serial.print("Gyro:");
  //  x = event->gyro.x;
  //  y = event->gyro.y;
   
    
    z = event->gyro.z;

 //y = event->acceleration.y;
    
  }


   else if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //Serial.print("Accl:");
    //x = event->acceleration.x;
    y = event->acceleration.y;
  //  y = event->acceleration.y;
  //  z = event->acceleration.z;
  }



  else {
    Serial.print("Unk:");
  }

return y;
}
