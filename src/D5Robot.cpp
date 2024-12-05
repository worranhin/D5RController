/**
 * @file D5Robot.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "D5Robot.h"

namespace D5R {

Joints JAWPOINT{}; // 钳口位置，需要实验确定

D5Robot::D5Robot() {}

D5Robot::D5Robot(
    const char *serialPort,
    std::string natorID,
    uint8_t topRMDID,
    uint8_t botRMDID,
    std::string topCameraID) {
    InitNator(natorID);
    InitRMD(serialPort, topRMDID, botRMDID);
    InitCamera(topCameraID);
}
D5Robot::~D5Robot() {
    if (topCamera != nullptr) {
        delete topCamera;
        topCamera = nullptr;
    }
    if (botCamera != nullptr) {
        delete botCamera;
        botCamera = nullptr;
    }
}

void D5Robot::InitNator(std::string natorID) {
    natorMotor = new NatorMotor(natorID);
}

/**
 * @brief Initializes the RMD motor with the given port name and IDs.
 *
 * This function sets up the RMD motor with the given port name and IDs.
 * The IDs are used to identify the motor when sending commands.
 *
 * @param portName The serial port name to use for the motor.
 * @param topRMDID The ID of the top RMD motor.
 * @param botRMDID The ID of the bottom RMD motor.
 */
void D5Robot::InitRMD(const char *portName, uint8_t topRMDID, uint8_t botRMDID) {
    _port = new SerialPort(portName);
    topRMDMotor = new RMDMotor(_port->GetHandle(), topRMDID);
    botRMDMotor = new RMDMotor(_port->GetHandle(), botRMDID);
}

void D5Robot::InitCamera(std::string topCameraId, std::string botCameraId) {
    topCamera = new CameraTop(topCameraId);
    botCamera = new CameraBot(botCameraId);
}

/**
 * @brief 将当前位置设为零点
 *
 */
void D5Robot::SetZero() {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::SetZero: natorMotor is not initialized. Call InitNator first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::SetZero: RMDMotor is not initialized. Call InitRMD first.");
    }

    natorMotor->SetZero();
    topRMDMotor->SetZero();
    botRMDMotor->SetZero();
}

/**
 * @brief 电机下电
 *
 */
void D5Robot::Stop() {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::Stop: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::Stop: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    natorMotor->Stop();
    topRMDMotor->Stop();
    botRMDMotor->Stop();
}

/**
 * @brief D5R绝对移动
 *
 * @param j 目标关节位置
 */
void D5Robot::JointsMoveAbsolute(const Joints j) {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveAbsolute: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveAbsolute: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    NTU_Point p{j.p2, j.p3, j.p4};
    natorMotor->GoToPoint_A(p);
    topRMDMotor->GoAngleAbsolute(j.r1);
    botRMDMotor->GoAngleAbsolute(j.r5);
}

/**
 * @brief D5R相对移动
 *
 * @param j 目标关节距离
 */
void D5Robot::JointsMoveRelative(const Joints j) {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveRelative: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveRelative: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    NTU_Point p{j.p2, j.p3, j.p4};
    natorMotor->GoToPoint_R(p);
    topRMDMotor->GoAngleRelative(j.r1);
    botRMDMotor->GoAngleRelative(j.r5);
}

/**
 * @brief 任务空间绝对位移
 *
 * @param ts 任务空间目标
 */
void D5Robot::TaskMoveAbsolute(const TaskSpace ts) {
    JointSpace js = KineHelper::Inverse(ts);
    JointsMoveAbsolute(js.ToControlJoint());
}

/**
 * @brief 任务空间相对位移
 *
 * @param ts 任务空间距离
 */
void D5Robot::TaskMoveRelative(const TaskSpace ts) {
    auto currentPose = GetCurrentPose();
    JointSpace deltaJoint = KineHelper::InverseDifferential(ts, currentPose);
    JointsMoveRelative(deltaJoint.ToControlJoint());
}

/**
 * @brief 获取当前关节位置
 *
 * @return Joints
 */
Joints D5Robot::GetCurrentJoint() {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::GetCurrentJoint: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::GetCurrentJoint: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    Joints j;

    j.r1 = topRMDMotor->GetSingleAngle_s();
    j.r5 = botRMDMotor->GetSingleAngle_s();

    NTU_Point np = this->natorMotor->GetPosition();
    j.p2 = np.x;
    j.p3 = np.y;
    j.p4 = np.z;

    return j;
}

/**
 * @brief 获取当前任务空间位置
 *
 * @return TaskSpace
 */
TaskSpace D5Robot::GetCurrentPose() {
    auto joint = GetCurrentJoint();
    JointSpace js(joint);
    auto ts = KineHelper::Forward(js);
    return ts;
}

/**
 * @brief 钳口视觉伺服控制
 *
 */
void D5Robot::VCJawChange() {
    // cv::Mat img;
    // if (!topCamera.Read(img)) {
    //     throw RobotException(ErrorCode::CameraReadError);
    // }
    if (!topCamera) {
        throw RobotException(ErrorCode::D5RCameraNotInitialized, "topCamera is not initialized");
    }

    std::vector<double> posError = topCamera->GetPhysicError();

    // 插入PID
    TaskSpace pError{-posError[1], -posError[0], 0, 0, posError[2]};
    JointSpace jError{};
    while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.01) {
        pError.Px = 0.2 * pError.Px;
        pError.Rz = 0.5 * pError.Rz;
        pError.Py = 0.4 * pError.Py;
        jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
        JointsMoveRelative(jError.ToControlJoint());
        Sleep(500);
        posError.clear();
        posError = topCamera->GetPhysicError();
        pError = {-posError[1], -posError[0], 0, 0, posError[2]};
    }
}

} // namespace D5R