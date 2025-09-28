#include "diffdrive_arduino/arduino_comms.h"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

void ArduinoComms::setup(const std::string &i2c_device, int32_t i2c_address,
                         int32_t /*timeout_ms*/) {
  fd_ = open(i2c_device.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open I2C device: " + i2c_device + " (" +
                             std::string(strerror(errno)) + ")");
  }

  i2c_address_ = i2c_address;
  if (ioctl(fd_, I2C_SLAVE, i2c_address_) < 0) {
    close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to set I2C address " +
                             std::to_string(i2c_address_));
  }
}

void ArduinoComms::sendEmptyMsg() { (void)sendMsg("\r"); }

void ArduinoComms::readEncoderValues(int &val_1, int &val_2) {
  std::string response = sendMsg("e\r");

  std::string delimiter = " ";
  size_t del_pos = response.find(delimiter);
  std::string token_1 = response.substr(0, del_pos);
  std::string token_2 = response.substr(del_pos + delimiter.length());

  val_1 = std::atoi(token_1.c_str());
  val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2) {
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2 << "\r";
  sendMsg(ss.str(), true);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o) {
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
  sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send,
                                  bool print_output) {
  if (fd_ < 0) {
    throw std::runtime_error("I2C device not opened");
  }

  // Write command
  ssize_t written = write(fd_, msg_to_send.c_str(), msg_to_send.size());
  if (written < 0) {
    throw std::runtime_error("I2C write failed: " +
                             std::string(strerror(errno)));
  }

  // Read fixed-size response
  char buffer[32];
  ssize_t n = read(fd_, buffer, sizeof(buffer));
  if (n < 0) {
    throw std::runtime_error("I2C read failed: " +
                             std::string(strerror(errno)));
  }

  // Always null terminate at the end
  buffer[sizeof(buffer) - 1] = '\0';

  // Build string from buffer
  std::string response(buffer);

  // Trim at first null, carriage return, or newline
  size_t pos = response.find_first_of("\r\n\0");
  if (pos != std::string::npos) {
    response = response.substr(0, pos);
  }

  if (print_output) {
    std::cerr << "Sent: " << msg_to_send << " (" << msg_to_send.size()
              << " bytes)"
              << " | Received: " << response << " (" << n << " bytes read)"
              << std::endl;
  }

  return response;
}
