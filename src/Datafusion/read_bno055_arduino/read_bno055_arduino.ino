#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SimpleTimer.h>
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SimpleTimer gryo_timer;
void read_gryo(){
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  SerialUSB.write("S");
  String data_now;
  
  data_now = String(millis()) + " " + String(orientationData.orientation.x) + " " + String(orientationData.orientation.y) + " " + String(orientationData.orientation.z) + " "
           + String(angVelocityData.acceleration.x) + " " + String(angVelocityData.acceleration.y) + " " + String(angVelocityData.acceleration.z) + " " 
           + String(linearAccelData.acceleration.x) + " " + String(linearAccelData.acceleration.y) + " " + String(linearAccelData.acceleration.z);
  SerialUSB.write(data_now.c_str());
  SerialUSB.write(" E");
  }
void setup()
{
  SerialUSB.begin(9600);
  gryo_timer.setInterval(100, read_gryo);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SerialUSB.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop()
{
gryo_timer.run();
}
