#pragma once

#include "JointSpace.hpp"
#include "KineHelper.hpp"

namespace D5R {

class TaskSpace {

public:
  double Px;
  double Py;
  double Pz;
  double Ry;
  double Rz;

  // JointSpace ToJointSpace() { KineHelper::Inverse(*this); }
};

} // namespace D5R
