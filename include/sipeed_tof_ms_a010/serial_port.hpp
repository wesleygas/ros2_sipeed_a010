#pragma once

#include <string>
#include <vector>
#include <memory>

namespace sipeed_tof
{

class SerialPort
{
public:
  explicit SerialPort(const std::string & dev_path);
  ~SerialPort();

  // Disable copy and assignment
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  bool is_open() const;
  bool send_command(const std::string & command, const std::string & expected_response, int timeout_ms = 100);
  void write_string(const std::string & data);
  std::string read_string();

private:
  bool configure_port();
  std::vector<uint8_t> read_bytes() const;
  void write_bytes(const std::vector<uint8_t> & data) const;
  void drain();

  std::string dev_path_;
  int fd_ = -1;
};

} // namespace sipeed_tof