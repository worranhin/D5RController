/**
 * @file NatorMotor.cpp
 * @author drawal (2581478521@qq.com)
 * @brief Implementation of NatorMotor class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "NatorMotor.h"
namespace D5R {

/**
 * @brief Construct a new Nator Motor:: Nator Motor object
 *
 * @param id Nator 电机 usbID
 */
NatorMotor::NatorMotor(std::string id) : _id(id) { Init(); }

/**
 * @brief Destroy the Nator Motor:: Nator Motor object
 *
 */
NatorMotor::~NatorMotor() { NT_CloseSystem(_handle); }

/**
 * @brief Nator 电机初始化
 *
 */
void NatorMotor::Init() {
    auto res = NT_OK;
    if (_id.empty()) {
        throw RobotException(ErrorCode::NatorInitError, "In NatorMotor::Init, device id is empty");
    }
    // 启动电机
    res = NT_OpenSystem(&_handle, _id.c_str(), "sync");
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorInitError, "In NatorMotor::Init, Failed to init device, error status: " + std::to_string(res));
    }
    // 允许手动控制
    res = NT_SetHCMEnabled(_handle, NT_HCM_ENABLED);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorSetError, "In NatorMotor::Init, Failed to set HCM enabled, error status: " + std::to_string(res));
    }
    // 启动编码器
    res = NT_SetSensorEnabled_S(_handle, NT_SENSOR_ENABLED);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorSetError, "In NatorMotor::Init, Failed to set sensor enabled, error status: " + std::to_string(res));
    }
}

/**
 * @brief 将当前位置设为零点
 *
 */
void NatorMotor::SetZero() {
    auto res = NT_OK;
    res = NT_SetPosition_S(_handle, NTU_AXIS_X, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorSetError, "In NatorMotor::SetZero, Failed to set axis_x zero, error status: " + std::to_string(res));
    }
    res = NT_SetPosition_S(_handle, NTU_AXIS_Y, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorSetError, "In NatorMotor::SetZero, Failed to set axis_y zero, error status: " + std::to_string(res));
    }
    res = NT_SetPosition_S(_handle, NTU_AXIS_Z, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorSetError, "In NatorMotor::SetZero, Failed to set axis_z zero, error status: " + std::to_string(res));
    }
}

/**
 * @brief 获取Nator电机当前位置
 *
 * @return NTU_Point p
 */
NTU_Point NatorMotor::GetPosition() {
    NTU_Point p;
    auto res = NT_OK;
    res = NT_GetPosition_S(_handle, NTU_AXIS_X, &(p.x));
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorGetError, "In NatorMotor::GetPosition, Failed to get axis_x positon, error status: " + std::to_string(res));
        return;
    }
    res = NT_GetPosition_S(_handle, NTU_AXIS_Y, &(p.y));
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorGetError, "In NatorMotor::GetPosition, Failed to get axis_y positon, error status: " + std::to_string(res));
        return;
    }
    res = NT_GetPosition_S(_handle, NTU_AXIS_Z, &(p.z));
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorGetError, "In NatorMotor::GetPosition, Failed to get axis_z positon, error status: " + std::to_string(res));
        return;
    }
    return p;
}

/**
 * @brief 电机绝对移动
 *
 * @param p 目标位置
 */
void NatorMotor::GoToPoint_A(NTU_Point p) {
    auto res = NT_OK;
    res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_X, p.x, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_A, Failed to move axis_x, error status: " + std::to_string(res));
    }
    res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_Y, p.y, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_A, Failed to move axis_y, error status: " + std::to_string(res));
    }
    res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_Z, p.z, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_A, Failed to move axis_z, error status: " + std::to_string(res));
    }
}

/**
 * @brief 阻塞函数
 *
 */
void NatorMotor::WaitUtilPositioned() {
    unsigned int res = 0;
    NT_GetStatus_S(_handle, NTU_AXIS_X, &res);
    while (res == NT_TARGET_STATUS) {
        Sleep(100);
        NT_GetStatus_S(_handle, NTU_AXIS_X, &res);
    }
    NT_GetStatus_S(_handle, NTU_AXIS_Y, &res);
    while (res == NT_TARGET_STATUS) {
        Sleep(100);
        NT_GetStatus_S(_handle, NTU_AXIS_Y, &res);
    }
    NT_GetStatus_S(_handle, NTU_AXIS_Z, &res);
    while (res == NT_TARGET_STATUS) {
        Sleep(100);
        NT_GetStatus_S(_handle, NTU_AXIS_Z, &res);
    }
}

/**
 * @brief 电机相对移动
 *
 * @param p 目标距离
 */
void NatorMotor::GoToPoint_R(NTU_Point p) {
    auto res = NT_OK;
    res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_X, p.x, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_R, Failed to move axis_x, error status: " + std::to_string(res));
    }
    res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_Y, p.y, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_R, Failed to move axis_y, error status: " + std::to_string(res));
    }
    res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_Z, p.z, 0);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::GoToPoint_R, Failed to move axis_z, error status: " + std::to_string(res));
    }
}

/**
 * @brief 电机下电
 *
 */
void NatorMotor::Stop() {
    auto res = NT_OK;
    res = NT_Stop_S(_handle, NTU_AXIS_X);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::Stop, Failed to stop axis_x, error status: " + std::to_string(res));
    }
    res = NT_Stop_S(_handle, NTU_AXIS_Y);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::Stop, Failed to stop axis_y, error status: " + std::to_string(res));
    }
    res = NT_Stop_S(_handle, NTU_AXIS_Z);
    if (res != NT_OK) {
        throw RobotException(ErrorCode::NatorMoveError, "In NatorMotor::Stop, Failed to stop axis_z, error status: " + std::to_string(res));
    }
}
} // namespace D5R