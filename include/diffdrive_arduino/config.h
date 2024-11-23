#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
  // Wheel names for the four wheels
  std::string front_left_wheel_name = "front_left_wheel";
  std::string front_right_wheel_name = "front_right_wheel";
  std::string back_left_wheel_name = "back_left_wheel";
  std::string back_right_wheel_name = "back_right_wheel";
  
  // Other configuration parameters
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
  float k_p = 30;
  float k_d = 20;
  float k_i = 0;
  float k_o = 100;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
