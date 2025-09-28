#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

class ArduinoComms
{
public:
  ArduinoComms() : fd_(-1), i2c_address_(0x08) {}

  ArduinoComms(const std::string &i2c_device, int32_t i2c_address, int32_t /*timeout_ms*/)
      : fd_(-1), i2c_address_(i2c_address)
  {
    setup(i2c_device, i2c_address, 0);
  }

  void setup(const std::string &i2c_device, int32_t i2c_address, int32_t timeout_ms);

  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return fd_ >= 0; }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = true);

private:
  int fd_;            ///< File descriptor for I²C device
  int i2c_address_;   ///< Arduino I²C address (e.g., 0x08)
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
