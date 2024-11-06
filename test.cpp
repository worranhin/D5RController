#include "D5Robot.h"

int main()
{
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
  // robot.JointsMoveRelative(j);
  // robot.Stop();
}