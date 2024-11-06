#include "SerialPort.h"
#include <stdexcept>

namespace D5R {
SerialPort::SerialPort(const char *serialPort) {
  _handle = CreateFile(serialPort, GENERIC_READ | GENERIC_WRITE, 0, 0,
                       OPEN_EXISTING, 0, 0);
  if (_handle == INVALID_HANDLE_VALUE) {
    throw std::runtime_error(std::string("Failed to open serial port"));
  }

  BOOL bSuccess = SetupComm(_handle, 100, 100);
  if (!bSuccess) {
    // ERROR_("Failed to init serial device buffer");
    throw "Failed to init serial device buffer";
  }

  COMMTIMEOUTS commTimeouts = {0};
  commTimeouts.ReadIntervalTimeout = 50;         // 读取时间间隔超时
  commTimeouts.ReadTotalTimeoutConstant = 100;   // 总读取超时
  commTimeouts.ReadTotalTimeoutMultiplier = 10;  // 读取超时乘数
  commTimeouts.WriteTotalTimeoutConstant = 100;  // 总写入超时
  commTimeouts.WriteTotalTimeoutMultiplier = 10; // 写入超时乘数

  bSuccess = SetCommTimeouts(_handle, &commTimeouts);
  if (!bSuccess) {
    // ERROR_("Failed to config Timeouts value");
    throw "Failed to config Timeouts value";
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(_handle, &dcbSerialParams)) {
    // ERROR_("Failed to obtain device comm status");
    throw "Failed to obtain device comm status";
  }
  dcbSerialParams.BaudRate = CBR_115200;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if (!SetCommState(_handle, &dcbSerialParams)) {
    // ERROR_("Failed to config DCB");
    throw "Failed to config DCB";
  }
}

SerialPort::~SerialPort() { CloseHandle(_handle); }

HANDLE SerialPort::GetHandle() { return _handle; }

} // namespace D5R
