#include "D5Robot.h"
#include "KineHelper.hpp"
#include <iostream>

std::ostream &printJointSpace(D5R::JointSpace js);
std::ostream &printTaskSpace(D5R::TaskSpace ts);
std::ostream &printSpace(D5R::JointSpace space);
std::ostream &printSpace(D5R::TaskSpace space);

int main() {
    using namespace D5R;

    D5R::JointSpace currentJointSpace(0.0, 0, 0, 0.0, 0.0);
    auto initJointSpace = currentJointSpace;
    printSpace(currentJointSpace) << std::endl;
    D5R::TaskSpace currentTaskSpace = KineHelper::Forward(currentJointSpace);

    D5R::TaskSpace deltaTaskSpace;
    deltaTaskSpace.Px = 0;
    deltaTaskSpace.Py = 0;
    deltaTaskSpace.Pz = 0;
    deltaTaskSpace.Ry = 0;
    deltaTaskSpace.Rz = 0.01;

    int iter = 1000;

    for (int i = 0; i < iter; i++) {
        // // 检测关节
        // printSpace(currentJointSpace) << "\t\t";
        // // 正解得到当前任务空间
        // currentTaskSpace = KineHelper::Forward(currentJointSpace);
        // // 加上微分得到下一任务空间
        // currentTaskSpace += deltaTaskSpace;
        // // 逆解得到下一关节空间
        // auto nextJointSpace = KineHelper::Inverse(currentTaskSpace);
        // // 计算差值得到相对运动大小
        // auto deltaJointSpace = nextJointSpace - currentJointSpace;
        // printSpace(deltaJointSpace) << std::endl;
        // // 更新当前关节空间
        // currentJointSpace += deltaJointSpace;

        // 输出关节量
        auto deltaJointSpace = D5R::KineHelper::InverseDifferential(deltaTaskSpace, currentTaskSpace);
        printSpace(deltaJointSpace) << "\t\t";
        currentJointSpace += deltaJointSpace;  // 移动关节
        currentTaskSpace = KineHelper::Forward(currentJointSpace);
        currentTaskSpace += deltaTaskSpace;
        currentJointSpace = KineHelper::Inverse(currentTaskSpace);
        printSpace(currentJointSpace) << std::endl;
    }

    printSpace(currentJointSpace) << std::endl;

    return 0;

    D5R::D5Robot robot("\\\\.\\COM16");

    for (int i = 0; i < 10; i++) {
        std::cout << "Round: " << i + 1 << std::endl;

        // 打印目前状态
        auto joint = robot.GetCurrentJoint();
        currentJointSpace.FromControlJoint(joint);
        std::cout << "currentJointSpace before move: ";
        printSpace(currentJointSpace);
        currentTaskSpace = KineHelper::Forward(currentJointSpace);
        std::cout << "currentTaskSpace before move: ";
        printSpace(currentTaskSpace);

        // 计算逆解
        auto targetTaskSpace = currentTaskSpace + deltaTaskSpace;
        auto targetJointSpace = D5R::KineHelper::Inverse(currentTaskSpace);
        std::cout << "deltaTaskSpace: ";
        printSpace(targetTaskSpace);
        std::cout << "deltaJointSpace: ";
        printSpace(targetJointSpace);

        // 移动
        
        // auto afterJs = currentJointSpace + deltaJointSpace;
        if (KineHelper::CheckJoint(targetJointSpace)) {
            robot.JointsMoveAbsolute(targetJointSpace.ToControlJoint());
            // currentJointSpace = afterJs;
            // currentTaskSpace = KineHelper::Forward(currentJointSpace);
        } else {
            std::cerr << "Joint out of range." << std::endl;
            break;
        }

        // 打印移动后状态
        Sleep(3000);
        joint = robot.GetCurrentJoint();
        currentJointSpace.FromControlJoint(joint);
        currentTaskSpace = KineHelper::Forward(currentJointSpace);
        std::cout << "currentTaskSpace after move: ";
        printSpace(currentTaskSpace);
        std::cout << "currentJointSpace after move: ";
        printSpace(currentJointSpace) << std::endl;
    }

        // for (int i = 0; i < 10; i++) {
    //     std::cout << "Round: " << i + 1 << std::endl;

    //     // 打印目前状态
    //     auto joint = robot.GetCurrentJoint();
    //     currentJointSpace.FromControlJoint(joint);
    //     std::cout << "currentJointSpace before move: ";
    //     printSpace(currentJointSpace);
    //     currentTaskSpace = KineHelper::Forward(currentJointSpace);
    //     std::cout << "currentTaskSpace before move: ";
    //     printSpace(currentTaskSpace);

    //     // 计算微分逆解
    //     auto deltaJointSpace = D5R::KineHelper::InverseDifferential(deltaTaskSpace, currentTaskSpace);
    //     std::cout << "deltaTaskSpace: ";
    //     printSpace(deltaTaskSpace);
    //     std::cout << "deltaJointSpace: ";
    //     printSpace(deltaJointSpace);

    //     // 模拟移动
    //     auto afterJs = currentJointSpace + deltaJointSpace;
    //     if (KineHelper::CheckJoint(afterJs)) {
    //         robot.JointsMoveRelative(deltaJointSpace.ToControlJoint());
    //         // currentJointSpace = afterJs;
    //         // currentTaskSpace = KineHelper::Forward(currentJointSpace);
    //     } else {
    //         std::cerr << "Joint out of range." << std::endl;
    //         break;
    //     }

    //     // 打印移动后状态
    //     Sleep(3000);
    //     joint = robot.GetCurrentJoint();
    //     currentJointSpace.FromControlJoint(joint);
    //     currentTaskSpace = KineHelper::Forward(currentJointSpace);
    //     std::cout << "currentTaskSpace after move: ";
    //     printSpace(currentTaskSpace);
    //     std::cout << "currentJointSpace after move: ";
    //     printSpace(currentJointSpace) << std::endl;
    // }

    return 0;
}

std::ostream &printJointSpace(D5R::JointSpace js) {
    return std::cout << js.R1 << " " << js.P2 << " " << js.P3 << " " << js.P4 << " " << js.R5;
}

std::ostream &printTaskSpace(D5R::TaskSpace ts) {
    return std::cout << ts.Px << " " << ts.Py << " " << ts.Pz << " " << ts.Ry << " " << ts.Rz;
}

std::ostream &printSpace(D5R::JointSpace space) {
    return printJointSpace(space);
}

std::ostream &printSpace(D5R::TaskSpace space) {
    return printTaskSpace(space);
}