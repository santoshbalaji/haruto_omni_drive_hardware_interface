#ifndef HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE__SERIAL_CONTROL_H__
#define HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE__SERIAL_CONTROL_H__

#include <string>

namespace haruto_omni_drive_hardware_interface
{
class SerialControl
{
private:
  const char * port_name_;
  int baud_rate_;
  int serial_port_id_;
  bool connected_;

public:
  SerialControl(char * port_name, int baud_rate);
  virtual ~SerialControl();
  bool open_connection();
  bool close_connection();
  bool check_connection_status();
  void read_message(char * buffer, int buffer_size);
  bool write_message(std::string message);
};
}

#endif // __HARUTO_MECANUM_HARDWARE__SERIAL_CONTROLLER_H__
