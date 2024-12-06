/**
 * @file RMDController.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief Implementation of RMDMotor class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "RMDMotor.h"
namespace D5R {

/**
 * @brief Construct a new RMDMotor::RMDMotor object
 *
 */
RMDMotor::RMDMotor() {}

/**
 * @brief Construct a new RMDMotor::RMDMotor object
 *
 * @param comHandle 串口句柄，由SerialPort获取
 * @param id 电机ID，如0x01， 0x02
 */
RMDMotor::RMDMotor(HANDLE comHandle, uint8_t id) : _id(id) {
    _handle = comHandle;
    GetPI();
}

/**
 * @brief Destroy the RMDMotor::RMDMotor object
 *
 */
RMDMotor::~RMDMotor() {}

/**
 * @brief 获取RMD多圈角度
 *
 * @return int64_t angle
 */
int64_t RMDMotor::GetMultiAngle_s() {
    uint8_t command[] = {0x3E, 0x92, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);
    const DWORD bytesToRead = 14;
    uint8_t readBuf[bytesToRead];
    int64_t motorAngle = 0;

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError, "In RMDMotor::GetMultiAngle_s, ");
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::GetMultiAngle_s, ");
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError, "In RMDMotor::GetMultiAngle_s, ");
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected, "In RMDMotor::GetMultiAngle_s, ");
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0x92 || readBuf[2] != _id ||
        readBuf[3] != 0x08 || readBuf[4] != (0x3E + 0x92 + _id + 0x08)) {
        throw RobotException(ErrorCode::RMDFormatError, "In RMDMotor::GetMultiAngle_s, ");
    }

    // check data sum
    if (_checksum(readBuf, 5, 13) != readBuf[13]) {
        throw RobotException(ErrorCode::RMDChecksumError, "In RMDMotor::GetMultiAngle_s, ");
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

    return motorAngle;
}

/**
 * @brief 获取RMD单圈角度
 *
 * @return uint16_t angle
 */
uint16_t RMDMotor::GetSingleAngle_s() {
    uint8_t command[5] = {0x3E, 0x94, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);

    const DWORD bytesToRead = 8;
    uint8_t readBuf[bytesToRead];

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError, "In RMDMotor::GetSingleAngle_s, ");
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::GetSingleAngle_s, ");
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError, "In RMDMotor::GetSingleAngle_s, ");
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected, "In RMDMotor::GetSingleAngle_s, ");
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0x94 || readBuf[2] != _id ||
        readBuf[3] != 0x02 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError, "In RMDMotor::GetSingleAngle_s, ");
    }

    // check data sum
    if (_checksum(readBuf, 5, 7) != readBuf[7]) {
        throw RobotException(ErrorCode::RMDChecksumError, "In RMDMotor::GetSingleAngle_s, ");
    }

    uint16_t circleAngle = 0;
    *(uint8_t *)(&circleAngle) = readBuf[5];
    *((uint8_t *)(&circleAngle) + 1) = readBuf[6];

    return circleAngle;
}

/**
 * @brief 帧头计算
 *
 * @param command 需计算的命令
 * @return uint8_t sum 累计帧头
 */
uint8_t RMDMotor::GetHeaderCheckSum(uint8_t *command) {
    uint8_t sum = 0x00;
    for (int i = 0; i < 4; ++i) {
        sum += command[i];
    }
    return sum;
}

/**
 * @brief RMD绝对转动
 *
 * @param angle 目标角度
 */
void RMDMotor::GoAngleAbsolute(int64_t angle) {
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
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::GoAngleAbsolute, ");
    }
}

/**
 * @brief RMD相对转动
 *
 * @param angle 目标角度
 */
void RMDMotor::GoAngleRelative(int64_t angle) {
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
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::GoAngleRelative, ");
    }
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
        throw RobotException(ErrorCode::SerialClearBufferError, "In RMDMotor::OpenLoopControl, ");
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::OpenLoopControl, ");
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError, "In RMDMotor::OpenLoopControl, ");
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected, "In RMDMotor::OpenLoopControl, ");
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0xA0 || readBuf[2] != _id ||
        readBuf[3] != 0x07 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError, "In RMDMotor::OpenLoopControl, ");
    }

    // check data sum
    if (_checksum(readBuf, 5, 12) != readBuf[12]) {
        throw RobotException(ErrorCode::RMDChecksumError, "In RMDMotor::OpenLoopControl, ");
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
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::SpeedControl, ");
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError, "In RMDMotor::SpeedControl, ");
    }

    // check received length
    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected, "In RMDMotor::SpeedControl, ");
    }

    // check received format
    if (readBuf[0] != 0x3E || readBuf[1] != 0xA2 || readBuf[2] != _id ||
        readBuf[3] != 0x07 || readBuf[4] != _checksum(readBuf, 0, 4)) {
        throw RobotException(ErrorCode::RMDFormatError, "In RMDMotor::SpeedControl, ");
    }

    // check data sum
    if (_checksum(readBuf, 5, 12) != readBuf[12]) {
        throw RobotException(ErrorCode::RMDChecksumError, "In RMDMotor::SpeedControl, ");
    }
}

/**
 * @brief RMD下电
 *
 */
void RMDMotor::Stop() {
    uint8_t command[] = {0x3E, 0x81, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);
    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::Stop, ");
    }
}

/**
 * @brief 将当前位置设置为电机零点
 *
 * @attention 该方法需要重新上电才能生效，且不建议频繁使用，会损害电机寿命。
 */
void RMDMotor::SetZero() {
    uint8_t command[] = {0x3E, 0x19, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);
    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::SetZero, ");
    }
}

/**
 * @brief 获取RMD PI参数
 *
 */
void RMDMotor::GetPI() {
    uint8_t command[] = {0x3E, 0X30, 0x00, 0x00, 0x00};
    command[2] = _id;
    command[4] = GetHeaderCheckSum(command);
    const DWORD bytesToRead = 12;
    uint8_t readBuf[bytesToRead];

    if (!PurgeComm(_handle, PURGE_RXCLEAR)) {
        throw RobotException(ErrorCode::SerialClearBufferError, "In RMDMotor::GetPI, ");
    }

    if (!WriteFile(_handle, command, sizeof(command), &_bytesWritten, NULL)) {
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::GetPI, ");
    }

    if (!ReadFile(_handle, readBuf, bytesToRead, &_bytesRead, NULL)) {
        throw RobotException(ErrorCode::SerialReceiveError, "In RMDMotor::GetPI, ");
    }

    if (_bytesRead != bytesToRead) {
        throw RobotException(ErrorCode::SerialReceiveError_LessThanExpected, "In RMDMotor::GetPI, ");
    }

    if (readBuf[0] != 0x3E || readBuf[1] != 0x30 || readBuf[2] != _id ||
        readBuf[3] != 0x06 || readBuf[4] != (0x3E + 0x30 + _id + 0x06)) {
        throw RobotException(ErrorCode::RMDFormatError, "In RMDMotor::GetPI, ");
    }

    if (_checksum(readBuf, 5, 11) != readBuf[11]) {
        throw RobotException(ErrorCode::RMDChecksumError, "In RMDMotor::GetPI, ");
    }

    _piParam.angleKp = (uint8_t)readBuf[5];
    _piParam.angleKi = (uint8_t)readBuf[6];
    _piParam.speedKp = (uint8_t)readBuf[7];
    _piParam.speedKi = (uint8_t)readBuf[8];
    _piParam.torqueKp = (uint8_t)readBuf[9];
    _piParam.torqueKi = (uint8_t)readBuf[10];
}

/**
 * @brief 改写电机PI
 *
 * @param arrPI 数组取前四位，分别是位置PI、速度PI，该型号电机无扭矩PI
 */
void RMDMotor::WriteAnglePI(const uint8_t *arrPI) {
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
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::WriteAnglePI, ");
    }
    GetPI();
}

/**
 * @brief 临时写入电机PI，断电失效，用于调试
 *
 * @param arrPI 数组取前四位，分别是位置PI、速度PI，该型号电机无扭矩PI
 */
void RMDMotor::DebugAnglePI(const uint8_t *arrPI) {
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
        throw RobotException(ErrorCode::SerialSendError, "In RMDMotor::DebugAnglePI, ");
    }
    GetPI();
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