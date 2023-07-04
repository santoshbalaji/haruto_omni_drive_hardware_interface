#ifndef __HARUTO_MECANUM_HARDWARE__SERIAL_CONTROL_H__
#define __HARUTO_MECANUM_HARDWARE__SERIAL_CONTROL_H__

namespace haruto_mecanum_hardware
{
  class SerialControl
  {
    private:
      
    public:
      SerialControl();
      virtual ~SerialControl();
      void open();
      void close();
      void check_open();
      void read();
      void write();
  };
}

#endif // __HARUTO_MECANUM_HARDWARE__SERIAL_CONTROLLER_H__
