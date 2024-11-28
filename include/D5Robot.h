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
    bool IsInit();
    bool SetZero();
    bool Stop();
    bool JointsMoveAbsolute(const Joints j);
    bool JointsMoveRelative(const Joints j);
    bool TaskMoveAbsolute(const TaskSpace ts);
    bool TaskMoveRelative(const TaskSpace ts);
    bool VCJawChange();

    Joints GetCurrentJoint();
    TaskSpace GetCurrentPose();

  private:
    inline static const std::string NatorId = "usb:id:7547982319";
    inline static const std::string UpCameraId = "00-21-49-03-4D-95";
    inline static const std::string BotCameraId = "00-21-49-03-4D-94";

    bool _isInit;
};
} // namespace D5R
