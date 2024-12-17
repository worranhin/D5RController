#include "DllApi.h"

const std::string port = "\\\\.\\COM16";

int main() {
    using namespace D5R;

    D5Robot* robot;
    CreateD5RobotInstance(robot, port.c_str());
    DestroyD5RobotInstance(robot);    
    
    return 0;
}