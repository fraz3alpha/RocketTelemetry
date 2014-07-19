
// Standard library includes
#include "BMP180.h"
// Project library includes
#include <Wire.h>

// Sensor objects
BMP180 pressure_sensor;

void setup() {
  // put your setup code here, to run once:
  
  // Initialise the serial port
  Serial.begin(38400);
  
  // Start the Wire library once, so all sensors can use it
  Wire.begin();
  
  // Initialise the BMP180 pressure sensor (defaults to ultra-high res - max 40Hz)
  pressure_sensor.begin();
  
  pressure_sensor.startContinuousSampling();
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Serial.println(pressure_sensor.readPressure());
  pressure_sensor.tick();
  Serial.println(pressure_sensor.readContinuousPressure());
}
