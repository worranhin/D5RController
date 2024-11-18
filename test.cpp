#include "D5Robot.h"
#include "DllApi.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"

D5R::D5Robot *pRobot;
const std::string port = "\\\\.\\COM14";
const std::string natorID = "usb:id:7547982319";

int TestMoving();
void TestKineHelper();
void TestApi();

int main() {
  // TestMoving();
  // TestApi();
  // TestKineHelper();

  return 0;
}

int TestMoving() {
  try {
    D5R::D5Robot robot(port.c_str(), natorID, 1, 2);
    D5R::Joints ja = {0, -13000000, 0, 0, 0};
    D5R::Joints jr = {200, 1000000, 0, 0, 6000};
    robot.JointsMoveAbsolute(ja);
    // robot.JointsMoveRelative(jr);
    // robot.Stop();
  } catch (const D5R::RobotException &e) {
    std::cout << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}

void TestApi() {
  D5R::ErrorCode ec;
  ec = CreateD5RobotInstance(pRobot, port.c_str(), natorID.c_str(), 1, 2);
  std::cout << ec << std::endl;
  // CallJointsMoveAbsolute(pRobot, {0, -13000000, 0, 0, 0});
  ec = CallJointsMoveRelative(pRobot, {-1000, 1000000, 2000000, 3000000, 1000});
  std::cout << ec << std::endl;
  // Sleep(1000);
  ec = DestroyD5RobotInstance(pRobot);
  std::cout << ec << std::endl;
}

void TestKineHelper() {
  D5R::JointSpace js = {0, 0, 0, 0, 0};
  D5R::TaskSpace ts = D5R::KineHelper::Forward(js);
  std::cout << ts.Px << " " << ts.Py << " " << ts.Pz << " " << ts.Ry << " "
            << ts.Rz << std::endl;
  D5R::JointSpace js2 = D5R::KineHelper::Inverse(ts);
  std::cout << js2.R1 << " " << js2.P2 << " " << js2.P3 << " " << js2.P4 << " "
            << js2.R5 << std::endl;
}