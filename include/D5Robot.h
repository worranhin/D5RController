#pragma once
#include "CameraUP.h"
#include "Joints.h"
#include "KineHelper.hpp"
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "RobotException.hpp"
#include "SerialPort.h"

namespace D5R {

struct Points {
    double px;
    double py;
    double pz;
    double ry;
    double rz;
};

class D5Robot {
  private:
    SerialPort *_port; // 该声明必须在 RMDMotor 声明之前

  public:
    NatorMotor *natorMotor;
    RMDMotor *topRMDMotor;
    RMDMotor *botRMDMotor;
    CameraUP *upCamera = nullptr;

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
    // Points FwKine(const Joints j);
    // Joints InvKine(const Points p);

  private:
    inline static const std::string NatorId = "usb:id:7547982319";
    inline static const std::string UpCameraId = "00-21-49-03-4D-95";
    inline static const std::string BotCameraId = "00-21-49-03-4D-94";

    bool _isInit;
};
} // namespace D5R
