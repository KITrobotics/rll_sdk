/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2019 Philipp Altoe <updim@student.kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_INVKINLIBRARY_H_
#define INCLUDE_INVKINLIBRARY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <iostream>   // for debugging
using namespace std;  // for debugging

// ros
#include <ros/ros.h>

#define NR_JOINTS 7

using namespace Eigen;

typedef enum {
  InvKin_OK = 0,
  InvKin_WARNING = 1 << 0,
  InvKin_ERROR = 1 << 1,
  InvKin_JOINTLIMIT = 1 << 2,
  InvKin_TARGET_TOO_FAR = 1 << 3,
  InvKin_TARGET_TOO_CLOSE = 1 << 4,
  InvKin_CLOSE_TO_SINGULARITY = 1 << 5,
  InvKin_SINGULARITY = 1 << 6,
  InvKin_NO_SOLUTION_FOR_ELBOW = 1 << 7
} InvKinMsg;

class InvKinJoints
{
public:
  InvKinJoints()
  {
    for (int i = 0; i < NR_JOINTS; i++)
    {
      j[i] = 0.0;
    }
  };

  // setters
  bool setJoints(const std::vector<double>& v);

  double& operator[](int i);

  double j[NR_JOINTS];
};

class InvKinFrame
{
public:
  InvKinFrame()
  {
    ori << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    pos << 0.0, 0.0, 0.0;
  };
  InvKinFrame(double d, double theta, double a, double alpha);  // construct frame from DH-parameters

  // getters/setters orientation
  void getQuaternion(double& w, double& x, double& y, double& z) const;
  void setQuaternion(double w, double x, double y, double z);

  // getters/setters position
  void setPosition(double x, double y, double z);

  // operators
  InvKinFrame operator*(InvKinFrame);

  Matrix3d ori;
  Vector3d pos;

private:
};

class InvKinXCart
{
public:
  InvKinXCart()
  {
    nsparam = 0;
    config = 0;
  };

  InvKinFrame pose;
  double nsparam;
  int config;
};

class InvKinElbowInterval
{
public:
  InvKinElbowInterval()
  {
    lower_limit = 0.0;
    upper_limit = 0.0;
    init = false;
    add = 0.0;
    overlap = false;
  };

  void setLimits(double lower, double upper)
  {
    lower_limit = lower;
    upper_limit = upper;
    init = true;
    add = 0.0;
  };
  void setAdd(double a)
  {
    add = a;
  };
  void reset()
  {
    lower_limit = 0.0;
    upper_limit = 0.0;
    init = false;
  };

  static void quickSortLower(InvKinElbowInterval intervals[], int left, int right);  // quicksort for lower_limit
  static void mergeSortedIntervals(InvKinElbowInterval intervals[],
                                   int n);  // merge intervals that are already sorted by quickSort_lower()

  static void determineBlockedIntervalsPivot(InvKinElbowInterval interval_limits[],
                                             InvKinElbowInterval blocked_intervals[], double& an, double& bn,
                                             double& cn, double& ad, double& bd, double& cd, int size);
  static void determineBlockedIntervalsHinge(InvKinElbowInterval interval_limits[],
                                             InvKinElbowInterval blocked_intervals[], double& a, double& b, double& c,
                                             double& gc_h, int size);

  static void mapLimitsToElbowAnglePivot(InvKinElbowInterval interval_limits[], double& lower_joint_limit,
                                         double& upper_joint_limit, double& an, double& bn, double& cn, double& ad,
                                         double& bd, double& cd, double& gc_p, int& size_init);
  static void mapLimitsToElbowAngleHinge(InvKinElbowInterval interval_limits[], double& lower_joint_limit,
                                         double& upper_joint_limit, double& a, double& b, double& c, double& gc_h,
                                         int& size_init);

  double derivativePivot(const double& an, const double& bn, const double& cn, const double& ad, const double& bd,
                         const double& cd);  // calculate derivative of joint-angle w.r.t elbow-angle
  double jointAnglePivot(const double& an, const double& bn, const double& cn, const double& ad, const double& bd,
                         const double& cd,
                         const double& GC);  // calculate joint-angle with elbow-angle set to lower_limit

  double derivativeHinge(const double& a, const double& b, const double& c, const double& GC);
  double jointAngleHinge(const double& a, const double& b, const double& c, const double& GC);

  bool operator<(InvKinElbowInterval) const;

  double lower_limit;
  double upper_limit;
  double add;    // additional variable, used in different methods
  bool overlap;  // determines if interval is overlapping from Pi to -Pi
  bool init;     // ElbowInterval initialized?
};

class InvKin
{
public:
  // Robot properties
  static double limbs_[4];
  static InvKinJoints lower_joint_limits_;
  static InvKinJoints upper_joint_limits_;
  static bool initialized_;

  // methods:

  // Compute FK:
  static InvKinMsg forwardKinematics(InvKinXCart& xCartPose, InvKinJoints& jointAngles);

  // Compute IK:
  static InvKinMsg inverseKinematics(InvKinJoints& jointAngles, InvKinXCart& xCartPose);
  // Compute IK and return helper matrices
  static InvKinMsg inverseKinematics(InvKinJoints& jointAngles, InvKinXCart& xCartPose, Matrix3d& As, Matrix3d& Bs,
                                     Matrix3d& Cs, Matrix3d& Aw, Matrix3d& Bw, Matrix3d& Cw, bool check_limits = false);

  // Initialize:
  bool initialize(const std::vector<double>& joint_distances, const std::vector<double>& lower_jointLimits,
                  const std::vector<double>& upper_jointLimits);

  // Compute feasible intervals in nullspace
  static InvKinMsg computeFeasibleIntervals(InvKinElbowInterval feasibleIntervals[], InvKinXCart& xCartPose,
                                            Matrix3d& As, Matrix3d& Bs, Matrix3d& Cs, Matrix3d& Aw, Matrix3d& Bw,
                                            Matrix3d& Cw, int& n);

  // get closest solution to seed-state using optimization defined in optimize()
  static InvKinMsg getClosestPositionIK(InvKinJoints sol[], int& index_sol, InvKinJoints& seed_state,
                                        InvKinXCart& seed_state_x, InvKinXCart& cartXPose, int configs[], int nConfigs,
                                        InvKinMsg (*optimize)(InvKinElbowInterval[], int&, InvKinXCart&, InvKinXCart&));

  // redundancy resolution using exponential function
  static InvKinMsg redundancyResolutionExp(InvKinElbowInterval intervals[], int& n, InvKinXCart& seed_state,
                                           InvKinXCart& cartXPose);

  // wrapper-methods for KinematicsBase-Plugin
  // redundancy resolution using e-function and hard coded config in ik_pose.config
  static InvKinMsg getIKefuncFixedConfig(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>& solution);
  // redundancy resolution using e-function and hard coded config and nsparam in ik_pose
  static InvKinMsg getIKefuncFixedConfigFixedNs(InvKinXCart ik_pose, InvKinJoints seed_state,
                                             std::vector<double>& solution);
  // redundancy-resolution using e-function
  static InvKinMsg getIKefunc(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>& solution);

private:
};

ostream& operator<<(ostream& stream, InvKinMsg const& val)
{
  stream << "InvKinError: ";

  if (val & InvKin_WARNING)
  {
    stream << "WARN ";
  }

  if (val & InvKin_ERROR)
  {
    stream << "ERROR ";
  }

  if (val & InvKin_JOINTLIMIT)
  {
    stream << "jointlimit ";
  }

  if (val & InvKin_TARGET_TOO_FAR)
  {
    stream << "target_too_far ";
  }

  if (val & InvKin_TARGET_TOO_CLOSE)
  {
    stream << "target_too_close ";
  }

  if (val & InvKin_CLOSE_TO_SINGULARITY)
  {
    stream << "close_to_singularity ";
  }

  if (val & InvKin_SINGULARITY)
  {
    stream << "singularity ";
  }

  if (val & InvKin_NO_SOLUTION_FOR_ELBOW)
  {
    stream << "no_solution_for_elbow ";
  }

  return stream;
}

#endif /* INCLUDE_INVKINLIBRARY_H_ */
