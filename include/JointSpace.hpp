#pragma once

#include "KineHelper.hpp"
#include "TaskSpace.hpp"
#include "Joints.h"

namespace D5R {

class JointSpace {
public:
  double R1;
  double P2;
  double P3;
  double P4;
  double R5;

  JointSpace() {}
  JointSpace(Joints joint) {
    R1 = joint.r1 / 100.0;
    P2 = joint.p2 / 1000000.0;
    P3 = joint.p3 / 1000000.0;
    P4 = joint.p4 / 1000000.0;
    R5 = joint.r5 / 100.0;
  }
  // JointSpace()

  Joints ToControlJoint() {
    Joints j;
    j.r1 = R1 * 100;
    j.p2 = P2 * 1000000;
    j.p3 = P3 * 1000000;
    j.p4 = P4 * 1000000;
    j.r5 = R5 * 100;

    return j;
  }

  void FromControlJoint(Joints j) {
    R1 = j.r1 / 100.0;
    P2 = j.p2 / 1000000.0;
    P3 = j.p3 / 1000000.0;
    P4 = j.p4 / 1000000.0;
    R5 = j.r5 / 100.0;
  }

  // TaskSpace ToTaskSpace() { return KineHelper::Forward(*this); }
};

} // namespace D5R