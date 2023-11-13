#include <haruto_omni_drive_hardware_interface/serial_control.hpp>

#include <iostream>
#include <fstream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace std;

namespace haruto_omni_drive_hardware_interface
{
  SerialControl::SerialControl(
    char * port_name,
    int baud_rate)
  {
    port_name_ = port_name;
    baud_rate_ = baud_rate;
  }

  SerialControl::~SerialControl()
  {
    delete port_name_;
    if (connected_)
    {
      close_connection();
    }
  }

  bool SerialControl::open_connection()
  {
    serial_port_id_ =  open(port_name_, O_RDWR);
    if (serial_port_id_ == -1) 
    {
        std::cout << "Error opening serial port." << endl;
        return false;
    }

    struct termios tty;
    tcgetattr(serial_port_id_, &tty);
    cfsetospeed(&tty, baud_rate_);
    cfsetispeed(&tty, baud_rate_);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tcsetattr(serial_port_id_, TCSANOW, &tty);

    connected_ = true;
    return true;
  }

  bool SerialControl::close_connection()
  {
    close(serial_port_id_);

    connected_ = false;
    return true;
  }

  void SerialControl::read_message(char * buffer, int buffer_size)
  {
    while(true)
    {
      ssize_t bytes_read = read(serial_port_id_, buffer, buffer_size - 1);
      if(bytes_read > 0)
      {
        buffer[bytes_read] = '\0';
      }
      else
      {
        break;
      }
    }
  }

  bool SerialControl::write_message(std::string message)
  {
    write(serial_port_id_, message.c_str(), message.size());
    return true;
  }

  bool SerialControl::check_connection_status()
  {
    return connected_;
  }
}