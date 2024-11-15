#include "D5Robot.h"
#include "KineHelper.hpp"

int TestMoving();
void TestKineHelper();
int main() {
  TestKineHelper();
  return 0;
}

int TestMoving() {
  std::string port = "\\\\.\\COM14";
  std::string natorID = "usb:id:7547982319";
  D5R::D5Robot robot(port.c_str(), natorID, D5R::ID_01, D5R::ID_02);
  if (!robot.IsInit())
  {
    ERROR_("Failed to init robot!");
    return -1;
  }

  D5R::Joints ja = {0, -13000000, 0, 0, -6000};
  D5R::Joints jr = {1000, -5000000, -5000000, -5000000, 1000};
  robot.JointsMoveAbsolute(ja);
  robot.JointsMoveRelative(jr);
  robot.Stop();

  return 0;
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