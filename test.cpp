#include "D5Robot.h"
#include "DllApi.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"

const std::string port = "\\\\.\\COM16";
// const std::string natorID = "usb:id:2250716012";

// int TestMoving();
// void TestKineHelper();
// void TestApi();

int main() {
    std::cout << "Hello Robot!" << std::endl;
    // TestMoving();
    // TestApi();
    // TestKineHelper();
    // D5R::D5Robot robot(port.c_str());
    D5R::D5Robot robot;

    try {
        robot.InitNator();
    } catch (D5R::RobotException &e) {
        std::cout << e.what() << std::endl;
    }

    try {
        robot.InitRMD(port.c_str());
    } catch (D5R::RobotException &e) {
        std::cout << e.what() << std::endl;
    }

    // try {
    //     robot.InitTopCamera();
    // } catch (D5R::RobotException &e) {
    //     std::cout << e.what() << std::endl;
    // }

    try {
        robot.InitBotCamera();
    } catch (D5R::RobotException &e) {
        std::cout << e.what() << std::endl;
    }

    // robot.JointsMoveAbsolute({0, 0, 8000000, 0, 0});
    // robot.Stop();
    // robot.topRMDMotor->GoAngleAbsolute(-17700);
    // robot.topRMDMotor->SetZero();
    // robot.Stop();
    // robot.Stop();
    // robot.SetZero();
    // robot.JointsMoveAbsolute({300, -3650000, 1800000, -10000000, 0}); // 别动

    // robot.Stop();
    // robot.JointsMoveAbsolute({0, 500000, 6000000, -7000000, 0});
    // Sleep(5000);
    // int64 start = cv::getTickCount();
    // robot.topCamera.GetPixelPos();
    // int64 end = cv::getTickCount();
    // int64 t = 1000.0 * (end - start) / cv::getTickFrequency();
    // std::cout << t << std::endl;
    // cv::waitKey(0);

    // robot.VCJawChange();
    // cv::waitKey(0);

    // cv::Mat img;
    // cv::Point2f roiP(800, 648);
    // robot.topCamera.Read(img);
    // std::vector<cv::Point2f> pos_jaw;
    // robot.topCamera.SIFT(img, JAW, pos_jaw);
    // float angle_jaw =
    //     atan2f(pos_jaw[1].y - pos_jaw[0].y, pos_jaw[1].x - pos_jaw[0].x) * (-180) / CV_PI;
    // cv::line(img, pos_jaw[0] + roiP, pos_jaw[1] + roiP, cv::Scalar(0), 4);
    // cv::putText(img, std::to_string(angle_jaw), pos_jaw[1] + roiP,
    //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 4);
    // std::string windowname = "image";
    // cv::namedWindow(windowname, cv::WINDOW_NORMAL);
    // cv::resizeWindow(windowname, cv::Size(1295, 1024));
    // cv::imshow(windowname, img);
    // cv::waitKey(0);

    // 测试相机
    std::string winname = "test";
    cv::namedWindow(winname, cv::WINDOW_NORMAL);
    cv::resizeWindow(winname, cv::Size(1295, 1024));
    int count = 10;

    // cv::Mat img_top;
    // while (robot.topCamera->Read(img_top)) {

    //     // cv::line(img_top, cv::Point(100, 1600), cv::Point(1800, 1600), cv::Scalar(0), 2);
    //     cv::imshow(winname, img_top);
    //     if (cv::waitKey(1) == 27) {
    //         break;
    //     }
    //     if (cv::waitKey(1) == 32) {
    //         // robot.topCamera.GetMapParam(img_top);
    //         std::string filename =
    //             "../image/12_6/topC_" + std::to_string(count++) + ".png";
    //         cv::imwrite(filename, img_top);
    //         // std::cout << count++ << std::endl;
    //         continue;
    //     }
    // }
    // cv::waitKey(0);

    cv::Mat img_bot;
    count = 0;
    while (robot.botCamera->Read(img_bot)) {
        // cv::line(img_bot, cv::Point(100, 1700), cv::Point(1800, 1700), cv::Scalar(255), 2);

        cv::imshow(winname, img_bot);
        if (cv::waitKey(1) == 27) {
            break;
        }
        if (cv::waitKey(1) == 32) {
            // robot.topCamera.GetMapParam(img_bot);
            std::string filename =
                "../image/12_6/botC_" + std::to_string(count++) + ".png";
            cv::imwrite(filename, img_bot);
            // std::cout << count++ << std::endl;
            continue;
        }
    }
    cv::waitKey(0);

    // Sleep(5000);
    // int64 start = cv::getTickCount();
    // robot.topCamera.GetPixelPos();
    // int64 end = cv::getTickCount();
    // int64 t = 1000.0 * (end - start) / cv::getTickFrequency();
    // std::cout << t << std::endl;
    // cv::waitKey(0);

    // robot.VCJawChange();
    // cv::waitKey(0);

    // 测试OpenCV
    //  cv::Mat img = cv::imread("../image/11_19/jaw_0.png");
    //  std::string windowname = "image";
    //  cv::namedWindow(windowname, cv::WINDOW_NORMAL);
    //  cv::resizeWindow(windowname, cv::Size(1295, 1024));
    //  cv::imshow(windowname, img);
    //  cv::waitKey(0);

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
