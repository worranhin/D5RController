/**
 * @file RMDController.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief Implementation of RMDMotor class
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "RMDMotor.h"
#include "LogUtil.h"

namespace D5R {

/**
 * Construct a RMDMotor object.
 *
 * @param serialPort The name of the serial port. e.g. "COM1"
 * @param id The ID of the motor. e.g. 0x01
 *
 * !! Deprecate !!
 * The constructor will try to initialize the serial port and get the PI
 * parameters of the motor. If the serial port is invalid, the constructor
 * will print an error message and set the _isInit flag to false.
 */
RMDMotor::RMDMotor(const char *serialPort, uint8_t id)
    : _serialPort(serialPort), _id(id) {
  _isInit = Init();
  GetPI();
  if (!_isInit) {
    ERROR_("Fail to init RMDMotor");
  }
}

/**
 * Construct a RMDMotor object using a handle to a serial port.
 *
 * @param comHandle The handle of the serial port.
 * @param id The ID of the motor. e.g. 0x01
 *
 * The constructor will not try to initialize the serial port. The caller
 * should ensure that the serial port is valid and the handle is a valid
 * handle to the serial port.
 */
RMDMotor::RMDMotor(HANDLE comHandle, uint8_t id)
    : _handle(comHandle), _id(id) {
      GetPI();
      _isInit = true;
    }

/**
 * Destructor of RMDMotor object.
 *
 * The destructor will close the handle of the serial port.
 */
RMDMotor::~RMDMotor() { CloseHandle(_handle); }

// 句柄初始化-----------------------------------------
bool RMDMotor::Init() {
  _handle = CreateFileA(_serialPort, GENERIC_READ | GENERIC_WRITE, 0, 0,
                        OPEN_EXISTING, 0, 0);
  if (_handle == INVALID_HANDLE_VALUE) {
    ERROR_("Invalid serialport");
    return false;
  }

  BOOL bSuccess = SetupComm(_handle, 100, 100);
  if (!bSuccess) {
    ERROR_("Failed to init serial device buffer");
    return false;
  }

  COMMTIMEOUTS commTimeouts = {0};
  commTimeouts.ReadIntervalTimeout = 50;         // 读取时间间隔超时
  commTimeouts.ReadTotalTimeoutConstant = 100;   // 总读取超时
  commTimeouts.ReadTotalTimeoutMultiplier = 10;  // 读取超时乘数
  commTimeouts.WriteTotalTimeoutConstant = 100;  // 总写入超时
  commTimeouts.WriteTotalTimeoutMultiplier = 10; // 写入超时乘数

  bSuccess = SetCommTimeouts(_handle, &commTimeouts);
  if (!bSuccess) {
    ERROR_("Failed to config Timeouts value");
    return false;
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(_handle, &dcbSerialParams)) {
    ERROR_("Failed to obtain device comm status");
    return false;
  }
  dcbSerialParams.BaudRate = CBR_115200;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if (!SetCommState(_handle, &dcbSerialParams)) {
    ERROR_("Failed to config DCB");
    return false;
  }

  return true;
}

// 是否初始化-----------------------------------------
bool RMDMotor::isInit() { return _isInit; }

// 设备重连------------------------------------------
bool RMDMotor::Reconnect() {
  if (_handle != nullptr) {
    CloseHandle(_handle);
  }
  return Init();
}

// 获取当前角度---------------------------------------
bool RMDMotor::GetMultiAngle_s(int64_t *angle) {
  uint8_t command[] = {0x3E, 0x92, 0x00, 0x00, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);
  const DWORD bytesToRead = 14;
  uint8_t readBuf[bytesToRead];
  int64_t motorAngle = 0;

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("GetMultiAngle_s: Failed to send command to device");
    return false;
  }

  if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
    ERROR_("GetMultiAngle_s: Failed to revice data from device");
    return false;
  }

  // check received length
  if (_bytesRead != bytesToRead) {
    ERROR_("GetMultiAngle_s: Abnormal received data - byte count");
    return false;
  }

  // check received format
  if (readBuf[0] != 0x3E || readBuf[1] != 0x92 || readBuf[2] != _id ||
      readBuf[3] != 0x08 || readBuf[4] != (0x3E + 0x92 + _id + 0x08)) {
    ERROR_("GetMultiAngle_s: Abnormal received data - frame header");
    return false;
  }

  // check data sum
  uint8_t sum = 0;
  for (int i = 5; i < 13; i++) {
    sum += readBuf[i];
  }
  if (sum != readBuf[13]) {
    ERROR_("GetMultiAngle_s: Abnormal received data - data");
    return false;
  }

  // motorAngle = readBuf[5] | (readBuf[6] << 8) | (readBuf[7] << 16) |
  // (readBuf[8] << 24);
  *(uint8_t *)(&motorAngle) = readBuf[5];
  *((uint8_t *)(&motorAngle) + 1) = readBuf[6];
  *((uint8_t *)(&motorAngle) + 2) = readBuf[7];
  *((uint8_t *)(&motorAngle) + 3) = readBuf[8];
  *((uint8_t *)(&motorAngle) + 4) = readBuf[9];
  *((uint8_t *)(&motorAngle) + 5) = readBuf[10];
  *((uint8_t *)(&motorAngle) + 6) = readBuf[11];
  *((uint8_t *)(&motorAngle) + 7) = readBuf[12];

  *angle = motorAngle;
  return true;
}

// 帧头计算------------------------------------------
uint8_t RMDMotor::GetHeaderCheckSum(uint8_t *command) {
  uint8_t sum = 0x00;
  for (int i = 0; i < 4; ++i) {
    sum += command[i];
  }
  return sum;
}

// 旋转角度-绝对---------------------------------------
bool RMDMotor::GoAngleAbsolute(int64_t angle) {
  int64_t angleControl = angle;
  uint8_t checksum = 0;

  uint8_t command[] = {0x3E, 0xA3, 0x00, 0x08, 0x00, 0xA0, 0x0F,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF};

  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);

  command[5] = *(uint8_t *)(&angleControl);
  command[6] = *((uint8_t *)(&angleControl) + 1);
  command[7] = *((uint8_t *)(&angleControl) + 2);
  command[8] = *((uint8_t *)(&angleControl) + 3);
  command[9] = *((uint8_t *)(&angleControl) + 4);
  command[10] = *((uint8_t *)(&angleControl) + 5);
  command[11] = *((uint8_t *)(&angleControl) + 6);
  command[12] = *((uint8_t *)(&angleControl) + 7);

  for (int i = 5; i < 13; i++) {
    checksum += command[i];
  }
  command[13] = checksum;

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("GoToAngle: Failed to send command to device");
    return false;
  }
  return true;
}

// 旋转角度-相对--------------------------------------
bool RMDMotor::GoAngleRelative(int64_t angle) {
  int64_t deltaAngle = angle;
  uint8_t checksum = 0;

  static uint8_t command[10] = {0x3E, 0xA7, 0x00, 0x04, 0x00, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);

  command[5] = *(uint8_t *)(&deltaAngle);
  command[6] = *((uint8_t *)(&deltaAngle) + 1);
  command[7] = *((uint8_t *)(&deltaAngle) + 2);
  command[8] = *((uint8_t *)(&deltaAngle) + 3);

  for (int i = 5; i < 9; i++) {
    checksum += command[i];
  }
  command[9] = checksum;

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("GoToAngle_R: Failed to send command to device");
    return false;
  }
  return true;
}

// 急停----------------------------------------------
bool RMDMotor::Stop() {
  uint8_t command[] = {0x3E, 0x81, 0x00, 0x00, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);
  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("Stop: Failed to send command to device");
    return false;
  }
  return true;
}

// 将当前位置设置为电机零点-----------------------------
// 注意：该方法需要重新上电才能生效，且不建议频繁使用，会损害电机寿命。
bool RMDMotor::SetZero() {
  uint8_t command[] = {0x3E, 0x19, 0x00, 0x00, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);
  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("SetZero: Failed to send command to device");
    return false;
  }
  return true;
}

// 获取PI参数-----------------------------------------
bool RMDMotor::GetPI() {
  uint8_t command[] = {0x3E, 0X30, 0x00, 0x00, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);
  const DWORD bytesToRead = 12;
  uint8_t readBuf[bytesToRead];

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("GetPI: Failed to send command to device");
    return false;
  }

  if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
    ERROR_("GetPI: Failed to revice data from device");
    return false;
  }

  if (_bytesRead != bytesToRead) {
    ERROR_("GetPI: Abnormal received data - byte count");
    return false;
  }

  if (readBuf[0] != 0x3E || readBuf[1] != 0x30 || readBuf[2] != _id ||
      readBuf[3] != 0x06 || readBuf[4] != (0x3E + 0x30 + _id + 0x06)) {
    ERROR_("GetPI: Abnormal received data - frame header");
    return false;
  }

  uint8_t sum = 0;
  for (int i = 5; i < 11; i++) {
    sum += readBuf[i];
  }
  if (sum != readBuf[11]) {
    ERROR_("GetPI: Abnormal received data - data");
    return false;
  }

  _piParam.angleKp = (uint8_t)readBuf[5];
  _piParam.angleKi = (uint8_t)readBuf[6];
  _piParam.speedKp = (uint8_t)readBuf[7];
  _piParam.speedKi = (uint8_t)readBuf[8];
  _piParam.torqueKp = (uint8_t)readBuf[9];
  _piParam.torqueKi = (uint8_t)readBuf[10];

  return true;
}

// 改写PI参数----------------------------------------
bool RMDMotor::WriteAnglePI(const uint8_t *arrPI) {
  uint8_t command[12] = {0x3E, 0x32, 0x00, 0x06, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);

  uint8_t checksum = 0;
  for (int i = 0; i < 6; i++) {
    command[5 + i] = (uint8_t)arrPI[i];
    checksum += command[5 + i];
  }
  command[11] = checksum;

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("WriteAnglePI: Failed to send command to device");
    return false;
  }
  if (!GetPI()) {
    ERROR_("Failed to updata PI param");
  }
  return true;
}

// 调试PI参数-------------------------------------------
bool RMDMotor::DebugAnglePI(const uint8_t *arrPI) {
  uint8_t command[12] = {0x3E, 0x31, 0x00, 0x06, 0x00};
  command[2] = _id;
  command[4] = GetHeaderCheckSum(command);
  uint8_t checksum = 0;
  for (int i = 0; i < 6; i++) {
    command[5 + i] = (uint8_t)arrPI[i];
    checksum += command[5 + i];
  }
  command[11] = checksum;

  if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
    ERROR_("DebugAnglePI: Failed to send command to device");
    return false;
  }
  if (!GetPI()) {
    ERROR_("Failed to updata PI param");
  }
  return true;
}

} // namespace D5R