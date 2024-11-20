#include "KineHelper.hpp"
#include <iostream>

std::ostream &printJointSpace(D5R::JointSpace js);
std::ostream &printTaskSpace(D5R::TaskSpace ts);
std::ostream &printSpace(D5R::JointSpace space);
std::ostream &printSpace(D5R::TaskSpace space);

int main() {
    using namespace D5R;

    D5R::JointSpace currentJointSpace(0.0, 0, 0, 0.0, 0.0);
    D5R::TaskSpace currentTaskSpace = KineHelper::Forward(currentJointSpace);

    D5R::TaskSpace deltaTaskSpace;
    deltaTaskSpace.Px = 0.7;
    deltaTaskSpace.Py = 0;
    deltaTaskSpace.Pz = 0;
    deltaTaskSpace.Ry = 0;
    deltaTaskSpace.Rz = 0.01;

    for (int i = 0; i < 50; i++) {
        std::cout << "Round: " << i + 1 << std::endl;

        // 打印目前状态
        std::cout << "currentTaskSpace before move: ";
        printSpace(currentTaskSpace);
        std::cout << "currentJointSpace before move: ";
        printSpace(currentJointSpace);

        // 计算微分逆解
        auto deltaJointSpace = D5R::KineHelper::InverseDifferential(deltaTaskSpace, currentTaskSpace);
        std::cout << "deltaTaskSpace: ";
        printSpace(deltaTaskSpace);
        std::cout << "deltaJointSpace: ";
        printSpace(deltaJointSpace);

        // 模拟移动
        auto afterJs = currentJointSpace + deltaJointSpace;
        if (KineHelper::CheckJoint(afterJs)) {
            currentJointSpace = afterJs;
            currentTaskSpace = KineHelper::Forward(currentJointSpace);
        } else {
            std::cerr << "Joint out of range." << std::endl;
            break;
        }

        // 打印移动后状态
        std::cout << "currentTaskSpace after move: ";
        printSpace(currentTaskSpace);
        std::cout << "currentJointSpace after move: ";
        printSpace(currentJointSpace) << std::endl;
    }

    return 0;
}

std::ostream &printJointSpace(D5R::JointSpace js) {
    return std::cout << js.R1 << " " << js.P2 << " " << js.P3 << " " << js.P4 << " " << js.R5 << std::endl;
}

std::ostream &printTaskSpace(D5R::TaskSpace ts) {
    return std::cout << ts.Px << " " << ts.Py << " " << ts.Pz << " " << ts.Ry << " " << ts.Rz << std::endl;
}

std::ostream &printSpace(D5R::JointSpace space) {
    return printJointSpace(space);
}

std::ostream &printSpace(D5R::TaskSpace space) {
    return printTaskSpace(space);
}