#include "sipeed_tof_ms_a010/serial_port.hpp"

#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

// Linux headers
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

namespace sipeed_tof
{

// Helper to set serial attributes
static bool serial_setup(int serial_port, unsigned int baudrate)
{
  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    throw std::runtime_error(std::string("Error from tcgetattr: ") + strerror(errno));
  }
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CREAD | CLOCAL);
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1; // 100ms timeout

  cfsetspeed(&tty, baudrate);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    throw std::runtime_error(std::string("Error from tcsetattr: ") + strerror(errno));
  }
  return true;
}

SerialPort::SerialPort(const std::string & dev_path) : dev_path_(dev_path)
{
  if (dev_path_.empty()) {
    throw std::invalid_argument("Serial device path cannot be empty.");
  }
  if (!configure_port()) {
    throw std::runtime_error("Failed to configure serial port: " + dev_path_);
  }
}

SerialPort::~SerialPort()
{
  if (fd_ != -1) {
    close(fd_);
  }
}

bool SerialPort::is_open() const
{
  return fd_ != -1;
}

bool SerialPort::configure_port()
{
  fd_ = open(dev_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    return false;
  }
  fcntl(fd_, F_SETFL, 0);
  return serial_setup(fd_, B115200);
}

void SerialPort::write_string(const std::string & data)
{
  write_bytes(std::vector<uint8_t>(data.begin(), data.end()));
}

std::string SerialPort::read_string()
{
  auto bytes = read_bytes();
  return std::string(bytes.begin(), bytes.end());
}

std::vector<uint8_t> SerialPort::read_bytes() const
{
  uint8_t read_buf[2048];
  ssize_t bytes_read = read(fd_, read_buf, sizeof(read_buf));
  if (bytes_read < 0) {
    return {};
  }
  return std::vector<uint8_t>(read_buf, read_buf + bytes_read);
}

void SerialPort::write_bytes(const std::vector<uint8_t> & data) const
{
  if (fd_ == -1) return;
  write(fd_, data.data(), data.size());
}

bool SerialPort::send_command(const std::string & command, const std::string & expected_response, int timeout_ms)
{
    write_string(command);
    auto start_time = std::chrono::steady_clock::now();
    std::string response;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() < timeout_ms)
    {
        response += read_string();
        if (response.find(expected_response) != std::string::npos)
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

} // namespace sipeed_tof