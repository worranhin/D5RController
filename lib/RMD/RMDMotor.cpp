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
#include "RobotException.hpp"

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
RMDMotor::RMDMotor() {}
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
RMDMotor::RMDMotor(HANDLE comHandle, uint8_t id) : _id(id) {
    _handle = comHandle;
    GetPI();
    _isInit = true;
}

/**
 * Destructor of RMDMotor object.
 *
 * The destructor will close the handle of the serial port.
 */
RMDMotor::~RMDMotor() {}

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

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError);
    }

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

uint16_t RMDMotor::GetSingleAngle_s() {
    uint8_t command[5] = {0x3E, 0x94, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);

    const DWORD bytesToRead = 8;
    uint8_t readBuf[bytesToRead];

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError);
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError);
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError);
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected);
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0x94 || readBuf[2] != _id ||
        readBuf[3] != 0x02 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError);
    }

    // check data sum
    if (_checksum(readBuf, 5, 7) != readBuf[7]) {
        throw RobotException(ErrorCode::RMDChecksumError);
    }

    uint16_t circleAngle = 0;
    *(uint8_t *)(&circleAngle) = readBuf[5];
    *((uint8_t *)(&circleAngle) + 1) = readBuf[6];

    return circleAngle;
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
        auto err = GetLastError();
        std::cerr << err << std::endl;
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

/**
 * @brief 开环控制
 *
 * This function sends a command to the motor to set its power level for open
 * loop control. It communicates with the motor via a serial interface, sending
 * a command packet and waiting for a response. The function checks for
 * communication errors and validates the response format and checksum.
 *
 * @param power 开环输出功率，数值范围 [-1000, 1000]
 *
 * @return The actual speed of the motor as reported by the motor in response,
 * represented as a 16-bit integer.
 *
 * @throw RobotException if there is an error in communication or if the
 * received data is malformed or fails checksum verification.
 */
int16_t RMDMotor::OpenLoopControl(int16_t power) {
    uint8_t command[8];
    command[0] = 0x3E;
    command[1] = 0xA0;
    command[2] = _id;
    command[3] = 0x02;
    command[4] = _checksum(command, 0, 4);
    command[5] = *(uint8_t *)(&power);
    command[6] = *((uint8_t *)(&power) + 1);
    command[7] = _checksum(command, 5, 7);

    const DWORD bytesToRead = 13;
    uint8_t readBuf[bytesToRead];

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError);
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError);
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError);
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected);
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0xA0 || readBuf[2] != _id ||
        readBuf[3] != 0x07 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError);
    }

    // check data sum
    if (_checksum(readBuf, 5, 12) != readBuf[12]) {
        throw RobotException(ErrorCode::RMDChecksumError);
    }

    int16_t speed = 0;
    *(uint8_t *)(&speed) = readBuf[8];
    *((uint8_t *)(&speed) + 1) = readBuf[9];

    return speed;
}

/**
 * @brief 速度控制
 * @param speed 电机速度，单位是 0.01 dps/LSB
 * @throw RobotException if the operation fails
 */
void RMDMotor::SpeedControl(int32_t speed) {
    uint8_t command[10];
    command[0] = 0x3E;
    command[1] = 0xA2;
    command[2] = _id;
    command[3] = 0x04;
    command[4] = _checksum(command, 0, 4);
    command[5] = *(uint8_t *)(&speed);
    command[6] = *((uint8_t *)(&speed) + 1);
    command[7] = *((uint8_t *)(&speed) + 2);
    command[8] = *((uint8_t *)(&speed) + 3);
    command[9] = _checksum(command, 5, 9);

    const DWORD bytesToRead = 13;
    uint8_t readBuf[bytesToRead];

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError);
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError);
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected);
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0xA2 || readBuf[2] != _id ||
        readBuf[3] != 0x07 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError);
    }

    // check data sum
    if (_checksum(readBuf, 5, 12) != readBuf[12]) {
        throw RobotException(ErrorCode::RMDChecksumError);
    }
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

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError);
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        ERROR_("GetPI: Failed to send command to device");
        // ERROR_(GetLastError)
        std::cerr << GetLastError() << std::endl;
        throw RobotException(ErrorCode::SerialSendError);
        return false;
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        ERROR_("GetPI: Failed to revice data from device");
        throw RobotException(ErrorCode::SerialReceiveError);
        return false;
    }

    if (_bytesRead != bytesToRead) {
        ERROR_("GetPI: Abnormal received data - byte count");
        throw RobotException(ErrorCode::RMDGetPIError);
        return false;
    }

    if (readBuf[0] != 0x3E || readBuf[1] != 0x30 || readBuf[2] != _id ||
        readBuf[3] != 0x06 || readBuf[4] != (0x3E + 0x30 + _id + 0x06)) {
        ERROR_("GetPI: Abnormal received data - frame header");
        throw RobotException(ErrorCode::RMDGetPIError);
        return false;
    }

    uint8_t sum = 0;
    for (int i = 5; i < 11; i++) {
        sum += readBuf[i];
    }
    if (sum != readBuf[11]) {
        ERROR_("GetPI: Abnormal received data - data");
        throw RobotException(ErrorCode::RMDGetPIError);
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

/**
 * \brief Calculate checksum of a given array segment
 * \param nums The array to calculate checksum
 * \param start The start index of the segment to calculate checksum (include)
 * \param end The end index of the segment to calculate checksum (not include)
 * \return The checksum of the given array segment
 */
uint8_t RMDMotor::_checksum(uint8_t nums[], int start, int end) {
    uint8_t sum = 0;
    for (int i = start; i < end; i++) {
        sum += nums[i];
    }

    return sum;
}

} // namespace D5R