#include "D5Robot.h"
#include "DllApi.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"

const std::string port = "\\\\.\\COM16";
const std::string natorID = "usb:id:7547982319";

// int TestMoving();
// void TestKineHelper();
// void TestApi();

int main() {
    // TestMoving();
    // TestApi();
    // TestKineHelper();
    D5R::D5Robot robot(port.c_str());
    // robot.Stop();
    // robot.JointsMoveAbsolute({300, 0, 0, -10000000, 0});
    // robot.JointsMoveAbsolute({0, 500000, 6000000, -7000000, 0});
    // robot.JointsMoveAbsolute({300, -2500000, 2100000, -7000000, 0});  //别动

    // int64 start = cv::getTickCount();
    // robot.upCamera.GetPixelPos();
    // int64 end = cv::getTickCount();
    // int64 t = 1000.0 * (end - start) / cv::getTickFrequency();
    // std::cout << t << std::endl;
    // cv::waitKey(0);

    // robot.VCJawChange();
    // cv::waitKey(0);

    // cv::Mat img;
    // std::string winname = "test";
    // cv::namedWindow(winname, cv::WINDOW_NORMAL);
    // cv::resizeWindow(winname, cv::Size(1295, 1024));
    // int count = 0;
    // while (robot.upCamera.Read(img)) {

    //     cv::imshow(winname, img);
    //     // cv::rectangle(img, cv::Poitn())
    //     if (cv::waitKey(1) == 27) {
    //         break;
    //     }
    //     if (cv::waitKey(1) == 32) {
    //         std::string filename =
    //             "../image/11_19/jaw_1" + std::to_string(count) + ".png";
    //         cv::imwrite(filename, img);
    //     }
    // }

    return 0;
}

int TestMoving() {
    // ***** Test Init ***** //
    try {
        D5R::D5Robot robot(port.c_str());
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
    D5R::D5Robot *pRobot;
    ec = CreateD5RobotInstance(pRobot, port.c_str());
    std::cout << ec << std::endl;
    // pRobot = CreateD5RobotInstance2(port.c_str(), natorID.c_str(), 1, 2);
    // CallJointsMoveAbsolute(pRobot, {0, -13000000, 0, 0, 0});
    ec = CallJointsMoveRelative(pRobot, {-1000, 1000000, 2000000, 3000000, 1000});
    std::cout << ec << std::endl;
    // Sleep(1000);
    ec = DestroyD5RobotInstance(pRobot);
    std::cout << ec << std::endl;
}

void TestKineHelper() {
    D5R::JointSpace js(0.0, 0.0, 0.0, 0.0, 0.0);
    D5R::TaskSpace ts = D5R::KineHelper::Forward(js);
    std::cout << ts.Px << " " << ts.Py << " " << ts.Pz << " " << ts.Ry << " "
              << ts.Rz << std::endl;
    D5R::JointSpace js2 = D5R::KineHelper::Inverse(ts);
    std::cout << js2.R1 << " " << js2.P2 << " " << js2.P3 << " " << js2.P4 << " "
              << js2.R5 << std::endl;
}
