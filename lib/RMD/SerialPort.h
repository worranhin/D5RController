/**
 * @file SerialPort.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief SerialPort Class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "RobotException.hpp"
#include <iostream>
#include <stdexcept>
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