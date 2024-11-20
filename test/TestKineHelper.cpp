#include "KineHelper.hpp"
#include <iostream>

int main() {
    D5R::TaskSpace currentTaskSpace;
    currentTaskSpace.Px = 0.0;
    currentTaskSpace.Py = 0.0;
    currentTaskSpace.Pz = 0.0;
    currentTaskSpace.Ry = 0.0;
    currentTaskSpace.Rz = 0.0;

    D5R::TaskSpace deltaTaskSpace;
    deltaTaskSpace.Px = 0.1;
    deltaTaskSpace.Py = 0.0;
    deltaTaskSpace.Pz = 0.0;
    deltaTaskSpace.Ry = 0.0;
    deltaTaskSpace.Rz = 0.0;

    auto jointMove = D5R::KineHelper::InverseDifferential(deltaTaskSpace, currentTaskSpace);

    std::cout << jointMove.R1 << " " << jointMove.P2 << " " << jointMove.P3 << " " << jointMove.P4 << " " << jointMove.R5 << std::endl;

    return 0;
}