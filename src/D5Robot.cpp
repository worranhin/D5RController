#include "D5Robot.h"

namespace D5R {

Joints JAWPOINT{}; // 钳口位置，需要实验确定

D5Robot::D5Robot() {
}

D5Robot::D5Robot(
    const char *serialPort,
    std::string natorID,
    uint8_t topRMDID,
    uint8_t botRMDID,
    std::string upCameraID) {
    InitNator(natorID);
    InitRMD(serialPort, topRMDID, botRMDID);
    InitCamera(upCameraID);
    // _isInit = natorMotor.IsInit();
    // if (!_isInit) {
    //     throw RobotException(ErrorCode::CreateInstanceError);
    // }
}
D5Robot::~D5Robot() {
    delete upCamera;
    upCamera = nullptr;
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

void D5Robot::InitCamera(std::string upCameraId) {
    upCamera = new CameraUP(upCameraId);
}

bool D5Robot::IsInit() { return _isInit; }

bool D5Robot::SetZero() {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::SetZero: natorMotor is not initialized. Call InitNator first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::SetZero: RMDMotor is not initialized. Call InitRMD first.");
    }

    if (!natorMotor->SetZero()) {
        ERROR_("In D5Robot::SetZero: Failed to set nator motor zero");
        return false;
    }
    if (!topRMDMotor->SetZero()) {
        ERROR_("In D5Robot::SetZero: Failed to set TOP RMD motor zero");
        return false;
    }
    if (!botRMDMotor->SetZero()) {
        ERROR_("In D5Robot::SetZero: Failed to set BOT RMD motor zero");
        return false;
    }
    return true;
}
bool D5Robot::Stop() {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::Stop: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::Stop: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    if (!natorMotor->Stop()) {
        ERROR_("Failed to stop nator motor");
        return false;
    }
    if (!topRMDMotor->Stop()) {
        ERROR_("Failed to stop TOP RMD motor");
        return false;
    }
    if (!botRMDMotor->Stop()) {
        ERROR_("Failed to stop BOT RMD motor");
        return false;
    }
    return true;
}

bool D5Robot::JointsMoveAbsolute(const Joints j) {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveAbsolute: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveAbsolute: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    NTU_Point p{j.p2, j.p3, j.p4};
    if (!natorMotor->GoToPoint_A(p)) {
        ERROR_("Failed to move nator motor");
        return false;
    }
    if (!topRMDMotor->GoAngleAbsolute(j.r1)) {
        ERROR_("Failed to move top RMD motor");
        return false;
    }
    if (!botRMDMotor->GoAngleAbsolute(j.r5)) {
        ERROR_("Failed to move bot RMD motor");
        return false;
    }
    return true;
}
bool D5Robot::JointsMoveRelative(const Joints j) {
    if (!natorMotor) {
        throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveRelative: natorMotor is not initialized. Call `InitNator` first.");
    }
    if (!topRMDMotor || !botRMDMotor) {
        throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveRelative: RMDMotor is not initialized. Call `InitRMD` first.");
    }

    NTU_Point p{j.p2, j.p3, j.p4};
    if (!natorMotor->GoToPoint_R(p)) {
        ERROR_("Failed to move nator motor");
        throw RobotException(ErrorCode::NatorMoveError);
        return false;
    }
    if (!topRMDMotor->GoAngleRelative(j.r1)) {
        ERROR_("Failed to move top RMD motor");
        throw RobotException(ErrorCode::RMDMoveError);
        return false;
    }
    if (!botRMDMotor->GoAngleRelative(j.r5)) {
        ERROR_("Failed to move bot RMD motor");
        throw RobotException(ErrorCode::RMDMoveError);
        return false;
    }
    return true;
}

bool D5Robot::TaskMoveAbsolute(const TaskSpace ts) {
    JointSpace js = KineHelper::Inverse(ts);
    return JointsMoveAbsolute(js.ToControlJoint());
}

bool D5Robot::TaskMoveRelative(const TaskSpace ts) {
    auto currentPose = GetCurrentPose();
    JointSpace deltaJoint = KineHelper::InverseDifferential(ts, currentPose);
    return JointsMoveRelative(deltaJoint.ToControlJoint());
}

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

    NTU_Point np;
    this->natorMotor->GetPosition(&np);
    j.p2 = np.x;
    j.p3 = np.y;
    j.p4 = np.z;

    return j;
}

TaskSpace D5Robot::GetCurrentPose() {
    auto joint = GetCurrentJoint();
    JointSpace js(joint);
    auto ts = KineHelper::Forward(js);
    return ts;
}

// Points D5Robot::FwKine(const Joints j) {
//   double l[5]{38, 11.5, 17.25, 28, 18.1};
//   Points p{};
//   p.px = (l[2] + l[4]) * sin(j.r1 * M_PI / 180.0) +
//          j.p3 * cos(j.r1 * M_PI / 180.0) + j.p2 * sin(j.r1 * M_PI / 180.0);
//   p.py = -(l[2] + l[4]) * cos(j.r1 * M_PI / 180.0) +
//          j.p3 * sin(j.r1 * M_PI / 180.0) - j.p2 * cos(j.r1 * M_PI / 180.0);
//   p.pz = -j.p4 - (l[0] + l[1] + l[3]);
//   p.ry = j.r1;
//   p.rz = j.r5;
//   return p;
// }

// Joints D5Robot::InvKine(const Points p) {
//   Joints j{};
//   j.r1 = p.ry;
//   j.r5 = p.rz;
//   j.p2 =
//       p.px * sin(j.r1 * M_PI / 180.0) - p.py * cos(j.r1 * M_PI / 180.0)
//       - 35.35;
//   j.p3 = p.px * cos(j.r1 * M_PI / 180.0) + p.py * sin(j.r1 * M_PI / 180.0);
//   j.p4 = -p.pz - 77.5;
//   return j;
// }

bool D5Robot::VCJawChange() {
    // cv::Mat img;
    // if (!upCamera.Read(img)) {
    //     throw RobotException(ErrorCode::CameraReadError);
    //     return false;
    // }
    if(!upCamera) {
        ERROR_("upCamera is not initialized");
        throw RobotException(ErrorCode::D5RCameraNotInitialized, "upCamera is not initialized");
        return false;
    }

    std::vector<double> posError = upCamera->GetPhysicError();

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
        posError = upCamera->GetPhysicError();
        pError = {-posError[1], -posError[0], 0, 0, posError[2]};
    }
    return true;
}

} // namespace D5R