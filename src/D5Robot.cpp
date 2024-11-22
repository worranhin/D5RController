#include "D5Robot.h"

namespace D5R {

Joints JAWPOINT{}; // 钳口位置，需要实验确定

D5Robot::D5Robot(
    const char *serialPort,
    std::string natorID,
    uint8_t topRMDID,
    uint8_t botRMDID,
    std::string upCameraID)
    : _port(serialPort),
      natorMotor(natorID),
      upCamera(upCameraID) {
    topRMDMotor._id = topRMDID;
    topRMDMotor._handle = _port.GetHandle();
    botRMDMotor._id = botRMDID;
    botRMDMotor._handle = _port.GetHandle();
    _isInit = natorMotor.IsInit() && upCamera.IsInit();
    if (!_isInit) {
        throw RobotException(ErrorCode::CreateInstanceError);
    }
}
D5Robot::~D5Robot() {}

bool D5Robot::IsInit() { return _isInit; }

bool D5Robot::SetZero() {
    if (!natorMotor.SetZero()) {
        ERROR_("Failed to set nator motor zero");
        return false;
    }
    if (!topRMDMotor.SetZero()) {
        ERROR_("Failed to set TOP RMD motor zero");
        return false;
    }
    if (!botRMDMotor.SetZero()) {
        ERROR_("Failed to set BOT RMD motor zero");
        return false;
    }
    return true;
}
bool D5Robot::Stop() {
    if (!natorMotor.Stop()) {
        ERROR_("Failed to stop nator motor");
        return false;
    }
    if (!topRMDMotor.Stop()) {
        ERROR_("Failed to stop TOP RMD motor");
        return false;
    }
    if (!botRMDMotor.Stop()) {
        ERROR_("Failed to stop BOT RMD motor");
        return false;
    }
    return true;
}

bool D5Robot::JointsMoveAbsolute(const Joints j) {
    NTU_Point p{j.p2, j.p3, j.p4};
    if (!natorMotor.GoToPoint_A(p)) {
        ERROR_("Failed to move nator motor");
        return false;
    }
    if (!topRMDMotor.GoAngleAbsolute(j.r1)) {
        ERROR_("Failed to move top RMD motor");
        return false;
    }
    if (!botRMDMotor.GoAngleAbsolute(j.r5)) {
        ERROR_("Failed to move bot RMD motor");
        return false;
    }
    return true;
}
bool D5Robot::JointsMoveRelative(const Joints j) {
    NTU_Point p{j.p2, j.p3, j.p4};
    if (!natorMotor.GoToPoint_R(p)) {
        ERROR_("Failed to move nator motor");
        throw RobotException(ErrorCode::NatorMoveError);
        return false;
    }
    if (!topRMDMotor.GoAngleRelative(j.r1)) {
        ERROR_("Failed to move top RMD motor");
        throw RobotException(ErrorCode::RMDMoveError);
        return false;
    }
    if (!botRMDMotor.GoAngleRelative(j.r5)) {
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
    Joints j;

    j.r1 = topRMDMotor.GetSingleAngle_s();
    j.r5 = botRMDMotor.GetSingleAngle_s();

    NTU_Point np;
    this->natorMotor.GetPosition(&np);
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

    std::vector<double> posError = upCamera.GetPhysicError();

    // 插入PID
    TaskSpace pError{-posError[1], -posError[0], 0, 0, posError[2]};
    JointSpace jError{};
    while (pError.Px > 0.1 || pError.Py > 0.1 || pError.Rz > 0.01) {
        pError.Px = 0.1 * pError.Px;
        pError.Rz = 0.25 * pError.Rz;
        jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
        JointsMoveRelative(jError.ToControlJoint());
        Sleep(2000);
        posError.clear();
        posError = upCamera.GetPhysicError();
        pError = {-posError[1], -posError[0], 0, 0, posError[2]};
    }
    return true;
}

} // namespace D5R