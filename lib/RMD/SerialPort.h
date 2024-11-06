#pragma once
#include <windows.h>

namespace D5R {
class SerialPort {
public:
  SerialPort(const char *serialPort);
  ~SerialPort();
  HANDLE GetHandle();

private:
  HANDLE _handle;
};

} // namespace D5R