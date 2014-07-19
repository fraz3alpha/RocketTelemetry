
// Project library includes
#include "BMP180.h"
#include "MPU6050.h"
// Project library includes
#include <Wire.h>

// Sensor objects
BMP180 pressure_sensor;
MPU6050 accel_gyro_sensor;

accel_t_gyro_value mpu6050_data;

void setup() {
  // put your setup code here, to run once:
  
  // Initialise the serial port
  Serial.begin(38400);
  
  // Start the Wire library once, so all sensors can use it
  Wire.begin();
  
  // Initialise the BMP180 pressure sensor (defaults to ultra-high res - max 40Hz)
  pressure_sensor.begin();
  // Initialisae the MPU6050 acceleration / gyro sensor (defaults to max scale)
  accel_gyro_sensor.begin();
  accel_gyro_sensor.configureMagnetometer();
  
  pressure_sensor.startContinuousSampling();
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Serial.println(pressure_sensor.readPressure());
  pressure_sensor.tick();
  mpu6050_data = accel_gyro_sensor.getData();
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(pressure_sensor.readContinuousPressure());
  Serial.print("\t");
  Serial.print(accel_gyro_sensor.getTemperature());
  Serial.print("\t");
  Serial.print(mpu6050_data.gyro.x);
  Serial.print("\t");
  Serial.print(mpu6050_data.gyro.y);
  Serial.print("\t");
  Serial.print(mpu6050_data.gyro.z);
  Serial.print("\t");
  Serial.print(mpu6050_data.accel.x);
  Serial.print("\t");
  Serial.print(mpu6050_data.accel.y);
  Serial.print("\t");
  Serial.print(mpu6050_data.accel.z);
  Serial.print("\t");
  Serial.print(mpu6050_data.mag.x);
  Serial.print("\t");
  Serial.print(mpu6050_data.mag.y);
  Serial.print("\t");
  Serial.print(mpu6050_data.mag.z);
  Serial.print("\t");
  Serial.println();
}
