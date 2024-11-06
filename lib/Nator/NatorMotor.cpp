/**
 * @file NatorMotor.cpp
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "NatorMotor.h"
namespace D5R {
// 构造析构---------------------------------------
NatorMotor::NatorMotor(std::string id) : _id(id) { 
  _isInit = Init();
  if(!_isInit)
    std::cerr << "Failed to init NatorMotor" << std::endl;
  }
NatorMotor::~NatorMotor() { NT_CloseSystem(_handle); }

// 初始化------------------------------------------
bool NatorMotor::Init() {
  auto res = NT_OK;
  if (_id.empty()) {
    std::cerr << "device id is empty" << std::endl;
  }
  res = NT_OpenSystem(&_handle, _id.c_str(), "sync");
  if (res != NT_OK) {
    std::cerr << "Failed to init device, error status: " << res << std::endl;
    return false;
  }
  res = NT_SetHCMEnabled(_handle, NT_HCM_ENABLED);
  if (res != NT_OK) {
    std::cerr << "Failed to set HCM enabled, error status: " << res
              << std::endl;
    return false;
  }
  res = NT_SetSensorEnabled_S(_handle, NT_SENSOR_ENABLED);
  if (res != NT_OK) {
    std::cerr << "Failed to set sensor enabled, error status: " << res
              << std::endl;
    return false;
  }
  _isInit = true;
  return true;
}

// 判断初始化成功------------------------------------
bool NatorMotor::IsInit() { return _isInit; }

// 设置零点-----------------------------------------
bool NatorMotor::SetZero() {
  auto res = NT_OK;
  res = NT_SetPosition_S(_handle, NTU_AXIS_X, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to set axis_x zero, error status: " << res
              << std::endl;
    return false;
  }
  res = NT_SetPosition_S(_handle, NTU_AXIS_Y, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to set axis_y zero, error status: " << res
              << std::endl;
    return false;
  }
  res = NT_SetPosition_S(_handle, NTU_AXIS_Z, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to set axis_z zero, error status: " << res
              << std::endl;
    return false;
  }
  return true;
}

// 获取当前位置
bool NatorMotor::GetPosition(NTU_Point *p) {
  auto res = NT_OK;
  res = NT_GetPosition_S(_handle, NTU_AXIS_X, &(p->x));
  if (res != NT_OK) {
    std::cerr << "Failed to get axis_x positon, error status: " << res
              << std::endl;
    return false;
  }
  res = NT_GetPosition_S(_handle, NTU_AXIS_Y, &(p->y));
  if (res != NT_OK) {
    std::cerr << "Failed to get axis_y positon, error status: " << res
              << std::endl;
    return false;
  }
  res = NT_GetPosition_S(_handle, NTU_AXIS_Z, &(p->z));
  if (res != NT_OK) {
    std::cerr << "Failed to get axis_z positon, error status: " << res
              << std::endl;
    return false;
  }
  return true;
}

// 绝对移动------------------------------------------
bool NatorMotor::GoToPoint_A(NTU_Point p) {
  auto res = NT_OK;
  res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_X, p.x, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_x, error status: " << res << std::endl;
    return false;
  }
  res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_Y, p.y, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_y, error status: " << res << std::endl;
    return false;
  }
  res = NT_GotoPositionAbsolute_S(_handle, NTU_AXIS_Z, p.z, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_z, error status: " << res << std::endl;
    return false;
  }
  //   WaitUtilPositioned();
  return true;
}

// 阻塞-------------------------------------------------
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

// 相对移动---------------------------------------------
bool NatorMotor::GoToPoint_R(NTU_Point p) {
  auto res = NT_OK;
  res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_X, p.x, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_x, error status: " << res << std::endl;
    return false;
  }
  res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_Y, p.y, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_y, error status: " << res << std::endl;
    return false;
  }
  res = NT_GotoPositionRelative_S(_handle, NTU_AXIS_Z, p.z, 0);
  if (res != NT_OK) {
    std::cerr << "Failed to move axis_z, error status: " << res << std::endl;
    return false;
  }
  //   WaitUtilPositioned();
  return true;
}

// 急停------------------------------------------------
bool NatorMotor::Stop() {
  auto res = NT_OK;
  res = NT_Stop_S(_handle, NTU_AXIS_X);
  if (res != NT_OK) {
    std::cerr << "Failed to stop axis_x, error status: " << res << std::endl;
    return false;
  }
  res = NT_Stop_S(_handle, NTU_AXIS_Y);
  if (res != NT_OK) {
    std::cerr << "Failed to stop axis_y, error status: " << res << std::endl;
    return false;
  }
  res = NT_Stop_S(_handle, NTU_AXIS_Z);
  if (res != NT_OK) {
    std::cerr << "Failed to stop axis_z, error status: " << res << std::endl;
    return false;
  }
  return true;
}
} // namespace D5R