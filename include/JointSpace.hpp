#pragma once

#include "KineHelper.hpp"
#include "TaskSpace.hpp"

namespace D5R {

class JointSpace {
public:
  double R1;
  double P2;
  double P3;
  double P4;
  double R5;

  // TaskSpace ToTaskSpace() { return KineHelper::Forward(*this); }
};

} // namespace D5R