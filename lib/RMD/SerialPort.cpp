/**
 * @file SerialPort.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief Implementation of SerialPort class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "SerialPort.h"
namespace D5R {
/**
 * @brief Construct a new Serial Port:: Serial Port object
 *
 * @param serialPort 串口
 * @attention 串口号如果超过10，输入格式将发生变化，例如窗口16，应输入 "////.//COM16"
 */
SerialPort::SerialPort(const char *serialPort) {
    _handle = CreateFile(serialPort, GENERIC_READ | GENERIC_WRITE, 0, 0,
                         OPEN_EXISTING, 0, 0);
    if (_handle == INVALID_HANDLE_VALUE) {
        throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to open serial port");
        return;
    }

    BOOL bSuccess = SetupComm(_handle, 100, 100);
    if (!bSuccess) {
        throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Setup Comm");
        return;
    }

    COMMTIMEOUTS commTimeouts = {0};
    commTimeouts.ReadIntervalTimeout = 50;         // 读取时间间隔超时
    commTimeouts.ReadTotalTimeoutConstant = 100;   // 总读取超时
    commTimeouts.ReadTotalTimeoutMultiplier = 10;  // 读取超时乘数
    commTimeouts.WriteTotalTimeoutConstant = 100;  // 总写入超时
    commTimeouts.WriteTotalTimeoutMultiplier = 10; // 写入超时乘数

    bSuccess = SetCommTimeouts(_handle, &commTimeouts);
    if (!bSuccess) {
        throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Set Comm Timeouts");
        return;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(_handle, &dcbSerialParams)) {
        throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Get Comm State");
        return;
    }
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(_handle, &dcbSerialParams)) {
        throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Set Comm State");
        return;
    }
}

SerialPort::~SerialPort() { CloseHandle(_handle); }

/**
 * @brief 返回串口句柄
 *
 * @return HANDLE
 */
HANDLE SerialPort::GetHandle() { return _handle; }

} // namespace D5R
