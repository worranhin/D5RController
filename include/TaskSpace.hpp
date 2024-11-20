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

  TaskSpace operator+= (const TaskSpace &rhs) {
    this->Px += rhs.Px;
    this->Py += rhs.Py;
    this->Pz += rhs.Pz;
    this->Ry += rhs.Ry;
    this->Rz += rhs.Rz;

    return *this;
  }

  // JointSpace ToJointSpace() { KineHelper::Inverse(*this); }
};

} // namespace D5R
