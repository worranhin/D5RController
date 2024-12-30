#pragma once
#include <windows.h>
#include <fileapi.h>
#include <WinBase.h>
#include <mutex>

namespace D5R {
class SerialPort {
private:
  HANDLE _handle;
  std::mutex _serialMutex;

public:
  SerialPort(const char *serialPort);
  ~SerialPort();
  HANDLE GetHandle();
  int write(const uint8_t* buffer, int size);
  int read(uint8_t* buffer, int size);
  bool writeAndRead(const uint8_t* writeBuffer, int writeSize, uint8_t* readBuffer, int readSize);
};

} // namespace D5R