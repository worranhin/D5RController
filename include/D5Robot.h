/**
 * @file D5Robot.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "CameraTop.h"
#include "KineHelper.hpp"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "SerialPort.h"

#include "LogUtil.h"
#include "RobotException.hpp"

namespace D5R {

class D5Robot {
  private:
    SerialPort *_port; // 该声明必须在 RMDMotor 声明之前

  public:
    NatorMotor *natorMotor;
    RMDMotor *topRMDMotor;
    RMDMotor *botRMDMotor;
    CameraTop *topCamera = nullptr;

    D5Robot();
    D5Robot(const char *serialPort, std::string natorID = NatorId,
            uint8_t topRMDID = 1, uint8_t botRMDID = 2,
            std::string upCameraID = UpCameraId);
    ~D5Robot();
    void InitNator(std::string natorID = NatorId);
    void InitRMD(const char *portName, uint8_t topRMDID = 1, uint8_t botRMDID = 2);
    void InitCamera(std::string upCameraId = UpCameraId);
    void SetZero();
    void Stop();
    void JointsMoveAbsolute(const Joints j);
    void JointsMoveRelative(const Joints j);
    void TaskMoveAbsolute(const TaskSpace ts);
    void TaskMoveRelative(const TaskSpace ts);
    void VCJawChange();

    Joints GetCurrentJoint();
    TaskSpace GetCurrentPose();

  private:
    inline static const std::string NatorId = "usb:id:7547982319";
    inline static const std::string UpCameraId = "00-21-49-03-4D-95";
    inline static const std::string BotCameraId = "00-21-49-03-4D-94";
};
} // namespace D5R
