#include "D5Robot.h"
#include "DllApi.h"
#include "RobotException.hpp"

D5R::D5Robot *pRobot;

int main()
{
  std::string port = "\\\\.\\COM14";
  std::string natorID = "usb:id:7547982319";

  // ***** Test Init ***** //
  // try {
  //   D5R::D5Robot robot(port.c_str(), natorID, 1, 2);
  //   // } catch (D5R::ErrorCode e) {
  //   //   std::cout << e << std::endl;
  // } catch (const D5R::RobotException &e) {
  //   std::cout << e.what() << std::endl;
  // } catch (const std::exception &e) {
  //   std::cout << e.what() << std::endl;
  // }

  // ***** Test API ***** //
  auto ec = CreateD5RobotInstance(pRobot, port.c_str(), natorID, 1, 2);
  std::cout << ec << std::endl;

  // if (!robot.IsInit())
  // {
  //   ERROR_("Failed to init robot!");
  //   return -1;
  // }

  // D5R::Joints ja = {0, -13000000, 0, 0, -6000};
  // D5R::Joints jr = {1000, -5000000, -5000000, -5000000, 1000};
  // robot->JointsMoveAbsolute(ja);
  // robot.JointsMoveRelative(j);
  // robot.Stop();
}