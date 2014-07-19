#include "MPU6050.h"

MPU6050::MPU6050() {
  
}

boolean MPU6050::begin() {
  // Default to maximum scale
  // Set the Acceleration full scale deflection to +/- 16g
  write_reg (MPU6050_I2C_ADDRESS,MPU6050_ACCEL_CONFIG, 0x18);
  // Set the Gyroscope full scale to +/-2000 degrees/sec
  write_reg (MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);
  
  // Clear the 'sleep' bit to start the sensor.
  write_reg (MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
}

void MPU6050::enablePassthrough(boolean enable) {
  if (enable) {
    // Allow the Arduino to directly access whatever is connected to the AUX I2C bus
    //  (probably a magnetometer)
    write_reg (MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, 0x2);
  } else {
    // Disable passthrough
    write_reg (MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, 0x0);
  }
}

accel_t_gyro_value MPU6050::getData() {
  read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  SWAP (accel_t_gyro.reg.x_mag_h, accel_t_gyro.reg.x_mag_l);
  SWAP (accel_t_gyro.reg.y_mag_h, accel_t_gyro.reg.y_mag_l);
  SWAP (accel_t_gyro.reg.z_mag_h, accel_t_gyro.reg.z_mag_l);
  
  return accel_t_gyro.value;
}

sensor_values MPU6050::getAcceleration() {
  return accel_t_gyro.value.accel;
}
sensor_values MPU6050::getGyro() {
  return accel_t_gyro.value.gyro;  
}
sensor_values MPU6050::getMagnetometer() {
  return accel_t_gyro.value.mag;  
}
int16_t MPU6050::getRawTemperature() {
  return accel_t_gyro.value.temperature; 
}
double MPU6050::getTemperature() {
  return ((double)accel_t_gyro.value.temperature + 12412.0) / 340.0;
}

void MPU6050::configureMagnetometer(uint8_t address) {
  enablePassthrough(true);
  // Initialise the HMC5883 sensor, continuous mode & appropriate scale
  write_reg(address, 0x00, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  write_reg(address, 0x01, 0xA0);
  // Set continuous mode
  write_reg(address, 0x02, 0x00);
  enablePassthrough(false);
  
  // Configure the MPU6050 to get data from the HMC5883 sensor
  
  // For each:
  // X axis word
  // Set Slave 0 address to 0x80 | 0x1E
  // Set Slave 0 x axis register
  // Set how many bytes, if we should switch them
  
  // Set slave address to 0x1E, OR'ed with 0x80 for Read mode
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV0_ADDR, 0x80 | 0x1E);
  // Set slave register (x_H is 0x03)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV0_REG, 0x03);
  // Setup
  // Enable slave
  // Data length of 2 (len 1 enabled)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV0_CTRL, (bit(MPU6050_I2C_SLV0_EN)|bit(MPU6050_I2C_SLV0_LEN1)));

  // Set slave address to 0x1E, OR'ed with 0x80 for Read mode
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV1_ADDR, 0x80 | 0x1E);
  // Set slave register (y_H is 0x05)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV1_REG, 0x05);
  // Setup
  // Enable slave
  // Data length of 2 (len 1 enabled)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV1_CTRL, (bit(MPU6050_I2C_SLV1_EN)|bit(MPU6050_I2C_SLV1_LEN1)));
 
  // Set slave address to 0x1E, OR'ed with 0x80 for Read mode
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV2_ADDR, 0x80 | 0x1E);
  // Set slave register (z_H is 0x07)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV2_REG, 0x07);
  // Setup
  // Enable slave
  // Data length of 2 (len 1 enabled)
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_I2C_SLV2_CTRL, (bit(MPU6050_I2C_SLV2_EN)|bit(MPU6050_I2C_SLV2_LEN1)));
  
  // Enable master mode
  write_reg(MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, (bit(MPU6050_I2C_MST_EN)));
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050::read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050::write(uint8_t address, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(address);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050::write_reg(uint8_t address, int reg, uint8_t data)
{
  int error;

  error = write(address, reg, &data, 1);

  return (error);
}
