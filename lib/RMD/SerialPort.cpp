#include "SerialPort.h"
#include "ErrorCode.h"
#include "RobotException.hpp"
#include <iostream>
#include <stdexcept>

namespace D5R {
SerialPort::SerialPort(const char *serialPort) {
  _handle = CreateFile(serialPort, GENERIC_READ | GENERIC_WRITE, 0, 0,
                       OPEN_EXISTING, 0, 0);
  if (_handle == INVALID_HANDLE_VALUE) {
    throw RobotException(ErrorCode::SerialInitError);
  }

  BOOL bSuccess = SetupComm(_handle, 100, 100);
  if (!bSuccess) {
    throw RobotException(ErrorCode::SerialInitError);
  }

  COMMTIMEOUTS commTimeouts = {0};
  commTimeouts.ReadIntervalTimeout = 10;         // 读取时间间隔超时(接收的两个字节之间的最大超时时间)
  commTimeouts.ReadTotalTimeoutConstant = 100;   // 总读取超时
  commTimeouts.ReadTotalTimeoutMultiplier = 1;  // 读取超时乘数
  commTimeouts.WriteTotalTimeoutConstant = 100;  // 总写入超时
  commTimeouts.WriteTotalTimeoutMultiplier = 1; // 写入超时乘数

  bSuccess = SetCommTimeouts(_handle, &commTimeouts);
  if (!bSuccess) {
    throw RobotException(ErrorCode::SerialInitError);
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(_handle, &dcbSerialParams)) {
    throw RobotException(ErrorCode::SerialInitError);
  }
  dcbSerialParams.BaudRate = CBR_115200;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if (!SetCommState(_handle, &dcbSerialParams)) {
    throw RobotException(ErrorCode::SerialInitError);
  }
}

SerialPort::~SerialPort() { CloseHandle(_handle); }

HANDLE SerialPort::GetHandle() { return _handle; }

/**
 * Writes data to the serial port.
 *
 * @param buffer A pointer to the data buffer to be written.
 * @param size The number of bytes to write from the buffer.
 * @return The number of bytes successfully written.
 * @throws RobotException if the write operation fails.
 */
int SerialPort::write(const uint8_t *buffer, int size) { 
  const std::lock_guard<std::mutex> lock(_serialMutex); // 锁住串口资源，避免多线程中出现问题

  DWORD bytesWritten = 0;
  if (!WriteFile(_handle, buffer, size, &bytesWritten, NULL)) {
    throw RobotException(ErrorCode::SerialSendError);
  }
  return bytesWritten;
 }

/**
 * Reads data from the serial port.
 *
 * @param buffer A pointer to the data buffer to store the read data.
 * @param size The number of bytes to read from the serial port.
 * @return The number of bytes successfully read.
 * @throws RobotException if the read operation fails.
 */
 int SerialPort::read(uint8_t *buffer, int size) { 
  const std::lock_guard<std::mutex> lock(_serialMutex); // 锁住串口资源
  DWORD bytesRead = 0;
  if (!ReadFile(_handle, buffer, size, &bytesRead, NULL)) {
    throw RobotException(ErrorCode::SerialReceiveError);
  }
  return bytesRead;
}

/**
 * Writes data to the serial port and reads data from the serial port.
 * 发送数据并等待接收数据,发送前会清空输入输出的缓冲区 (阻塞)
 *
 * @param writeBuffer A pointer to the data buffer to be written.
 * @param writeSize The number of bytes to write from the buffer.
 * @param readBuffer A pointer to the data buffer to store the read data.
 * @param readSize The number of bytes to read from the serial port.
 * @return true if the write and read operations are successful, false otherwise.
 */
bool SerialPort::writeAndRead(const uint8_t *writeBuffer, int writeSize,
                              uint8_t *readBuffer, int readSize) {
  const std::lock_guard<std::mutex> lock(_serialMutex); // 锁住串口资源
  DWORD bytesWritten = 0;
  DWORD bytesRead = 0;

  PurgeComm(_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);  

  if (!WriteFile(_handle, writeBuffer, writeSize, &bytesWritten, NULL)) {
    // throw RobotException(ErrorCode::SerialSendError);
    return false;
  }
  if (writeSize != bytesWritten) {
    return false;
  }

  if (!ReadFile(_handle, readBuffer, readSize, &bytesRead, NULL)) {
    // throw RobotException(ErrorCode::SerialReceiveError);
    return false;
  }
  if (readSize != bytesRead) {
    // throw RobotException(ErrorCode::SerialReceiveError);
    return false;
  }
  
  return true;
}

} // namespace D5R
