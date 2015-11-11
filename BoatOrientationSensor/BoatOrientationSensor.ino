#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* 
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground opkokokokok
*/


#define BNO055_SAMPLERATE_DELAY_MS (50)
   
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void) 
{
  Serial.begin(9600);  
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
}


void loop(void) 
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);



// absolute position in degrees
float heading = atan2(magnet.x(), magnet.y());
Serial.println(heading* (57296 / 1000));

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
