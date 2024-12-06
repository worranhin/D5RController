#pragma once

#include "JointSpace.hpp"
#include "TaskSpace.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>

namespace D5R {

class KineHelper {
  private:
    inline static const double l1 = 38.0;
    inline static const double l2 = 11.5 + 1.5;
    inline static const double l3 = 17.25;
    inline static const double l4 = 28.0;
    inline static const double l5 = 29.0;
    inline static const double ltx = 72.9;
    inline static const double lty = 42.5;
    inline static const double ltz = 9.46;

    inline static const double _R1min = -90.0, _R1max = 90.0;
    inline static const double _R5min = -45.0, _R5max = 90.0;
    inline static const double _P2min = -15.0, _P2max = 15.0;
    inline static const double _P3min = -15.0, _P3max = 15.0;
    inline static const double _P4min = -15.0, _P4max = 15.0;

  public:
    static TaskSpace Forward(const JointSpace &space) {
        double m1 = l3 + l5 + lty + space.P2;

        if (!CheckJoint(space)) {
            throw std::out_of_range("In KineHelper::Forward: Joint out of range.");
        }

        TaskSpace ts;
        ts.Px = m1 * Sind(space.R1) + space.P3 * Cosd(space.R1) +
                ltx * Cosd(space.R1) * Cosd(space.R5) +
                ltz * Cosd(space.R1) * Sind(space.R5);

        ts.Py = space.P3 * Sind(space.R1) - m1 * Cosd(space.R1) +
                ltx * Sind(space.R1) * Cosd(space.R5) +
                ltz * Sind(space.R1) * Sind(space.R5);

        ts.Pz =
            ltx * Sind(space.R5) - ltz * Cosd(space.R5) - space.P4 - (l1 + l2 + l4);

        ts.Ry = -space.R5;
        ts.Rz = space.R1;

        roundSpace(ts);
        return ts;
    }

    static JointSpace Inverse(const TaskSpace &space) {
        double m1 = l3 + l5 + lty;
        double m2 = l1 + l2 + l4;

        JointSpace js;
        js.R1 = space.Rz;
        js.R5 = -space.Ry;
        js.P2 = space.Px * Sind(space.Rz) - space.Py * Cosd(space.Rz) - m1;
        js.P3 = space.Px * Cosd(space.Rz) + space.Py * Sind(space.Rz) - ltx * Cosd(-space.Ry) - ltz * Sind(-space.Ry);
        js.P4 = -space.Pz + ltx * Sind(-space.Ry) - ltz * Cosd(-space.Ry) - m2;

        roundSpace(js);

        return js;
    }

    static JointSpace InverseDifferential(const TaskSpace &deltaSpace, const TaskSpace &currentSpace) {
        using namespace std::numbers;
        JointSpace djs;

        double dRz = deltaSpace.Rz * pi / 180.0; // !!!计算时需要用弧度值!!!
        double dRy = deltaSpace.Ry * pi / 180.0;
        double dPx = deltaSpace.Px * pi;
        double dPy = deltaSpace.Py * pi;
        double dPz = deltaSpace.Pz * pi;

        double rz = currentSpace.Rz;
        double ry = currentSpace.Ry;
        double px = currentSpace.Px;
        double py = currentSpace.Py;
        double pz = currentSpace.Pz;

        djs.R1 = dRz * 180.0 / pi;
        djs.R5 = -dRy * 180.0 / pi;
        djs.P2 = Sind(rz) * dPx - Cosd(rz) * dPy + (px * Cosd(rz) + py * Sind(rz)) * dRz;
        djs.P3 = Cosd(rz) * dPx + Sind(rz) * dPy + (-px * Sind(rz) + py * Cosd(rz)) * dRz + (ltx * Sind(ry) + ltz * Cosd(ry)) * dRy;
        djs.P4 = -dPz + (ltx * Cosd(ry) - ltz * Sind(ry)) * (-dRy);
        // djs.P2 = Sind(rz) * dPx + px * Cosd(rz) * dRz - (Cosd(rz) * dPy - py * Sind(rz) * dRz);
        // djs.P3 = Cosd(rz) * dPx - px * Sind(rz) * dRz + Sind(rz) * dPy + py * Cosd(rz) * dRz + ltx * Sind(ry) * dRy + ltz * Cosd(ry) * dRy;
        // djs.P4 = -dPz - ltx * Cosd(ry) * dRy + ltz * Sind(ry) * dRy;

        roundSpace(djs);

        return djs;
    }

    static bool CheckJoint(const JointSpace &js) {
        // bool good1 = js.R1 >= _R1min && js.R1 <= _R1max;
        bool good2 = js.P2 >= _P2min && js.P2 <= _P2max;
        bool good3 = js.P3 >= _P3min && js.P3 <= _P3max;
        bool good4 = js.P4 >= _P4min && js.P4 <= _P4max;
        // bool good5 = js.R5 >= _R5min && js.R5 <= _R5max;

        return good2 && good3 && good4;
    }

    static JointSpace ClipJoint(const JointSpace &js) {
        JointSpace clipped;
        clipped.R1 = std::clamp(js.R1, _R1min, _R1max);
        clipped.P2 = std::clamp(js.P2, _P2min, _P2max);
        clipped.P3 = std::clamp(js.P3, _P3min, _P3max);
        clipped.P4 = std::clamp(js.P4, _P4min, _P4max);
        clipped.R5 = std::clamp(js.R5, _R5min, _R5max);

        return clipped;
    }

    static bool CheckJoint(const JointSpace &js, bool which[5]) {
        bool good1 = js.R1 >= _R1min && js.R1 <= _R1max;
        bool good2 = js.P2 >= _P2min && js.P2 <= _P2max;
        bool good3 = js.P3 >= _P3min && js.P3 <= _P3max;
        bool good4 = js.P4 >= _P4min && js.P4 <= _P4max;
        bool good5 = js.R5 >= _R5min && js.R5 <= _R5max;

        which[0] = good1;
        which[1] = good2;
        which[2] = good3;
        which[3] = good4;
        which[4] = good5;

        return good1 && good2 && good3 && good4 && good5;
    }

  private:
    static double Cosd(double x) {
        using namespace std::numbers;
        return std::cos(x * pi / 180.0);
    }

    static double Sind(double x) {
        using namespace std::numbers;
        return std::sin(x * pi / 180.0);
    }

    static void roundSpace(JointSpace &jointSpace) {
        jointSpace.R1 = std::round(jointSpace.R1 * 100) / 100; // 最小 0.01 deg
        jointSpace.R5 = std::round(jointSpace.R5 * 100) / 100;
        jointSpace.P2 = std::round(jointSpace.P2 * 10000) / 10000; // 最小 100 nm
        jointSpace.P3 = std::round(jointSpace.P3 * 10000) / 10000;
        jointSpace.P4 = std::round(jointSpace.P4 * 10000) / 10000;
    }

    static void roundSpace(TaskSpace &taskSpace) {
        taskSpace.Px = std::round(taskSpace.Px * 10000) / 10000; // 保留至 100 nm
        taskSpace.Py = std::round(taskSpace.Py * 10000) / 10000;
        taskSpace.Pz = std::round(taskSpace.Pz * 10000) / 10000;
        taskSpace.Ry = std::round(taskSpace.Ry * 100) / 100; // 保留至 0.01 deg
        taskSpace.Rz = std::round(taskSpace.Rz * 100) / 100;
    }
};

} // namespace D5R