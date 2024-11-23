#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstring>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(long &val_1, long &val_2, long &val_3, long &val4);
  void setMotorValues(int val_1, int val_2, int val_3, int val_4);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);
  void resetEncoder();
  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;  ///< Underlying serial connection 
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H