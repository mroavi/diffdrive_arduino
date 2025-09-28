#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;

  // For I2C communication
  std::string device = "/dev/i2c-1";  // typical I2C bus on Linux SBCs
  int i2c_address = 0x08;             // Arduino I2C slave address
  int timeout = 1000;

  int enc_counts_per_rev = 1920;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
