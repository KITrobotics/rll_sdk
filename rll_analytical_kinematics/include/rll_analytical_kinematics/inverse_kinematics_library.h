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
#include <iostream>

// ros
#include <ros/ros.h>

#define NR_JOINTS 7

// using namespace Eigen;

enum InvKinMsg
{
  INVKIN_OK = 0,
  INVKIN_WARNING = 1 << 0,
  INVKIN_ERROR = 1 << 1,
  INVKIN_JOINTLIMIT = 1 << 2,
  INVKIN_TARGET_TOO_FAR = 1 << 3,
  INVKIN_TARGET_TOO_CLOSE = 1 << 4,
  INVKIN_CLOSE_TO_SINGULARITY = 1 << 5,
  INVKIN_SINGULARITY = 1 << 6,
  INVKIN_NO_SOLUTION_FOR_ELBOW = 1 << 7
};

class InvKinJoints
{
public:
  InvKinJoints()
  {
    for (auto& joint_angle : j)
    {
      joint_angle = 0.0;
    }
  };

  // setters
  bool setJoints(const std::vector<double>& v);
  double& operator[](int i);

  // getters
  const double operator()(int i) const;

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
  void getQuaternion(double* w, double* x, double* y, double* z) const;
  void setQuaternion(double w, double x, double y, double z);

  // getters/setters position
  void setPosition(double x, double y, double z);

  // operators
  InvKinFrame operator*(InvKinFrame t) const;

  Eigen::Matrix3d ori;
  Eigen::Vector3d pos;

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

  static void mergeSortedIntervals(InvKinElbowInterval intervals[],
                                   int n);  // merge intervals that are already sorted by quickSort_lower()

  static void determineBlockedIntervalsPivot(InvKinElbowInterval interval_limits[],
                                             InvKinElbowInterval blocked_intervals[], double an, double bn, double cn,
                                             double ad, double bd, double cd, int size);
  static void determineBlockedIntervalsHinge(InvKinElbowInterval interval_limits[],
                                             InvKinElbowInterval blocked_intervals[], double a, double b, double c,
                                             double gc_h, int size);

  static void mapLimitsToElbowAnglePivot(InvKinElbowInterval interval_limits[], double lower_joint_limit,
                                         double upper_joint_limit, double an, double bn, double cn, double ad,
                                         double bd, double cd, double gc_p, int* size_init);
  static void mapLimitsToElbowAngleHinge(InvKinElbowInterval interval_limits[], double lower_joint_limit,
                                         double upper_joint_limit, double a, double b, double c, double gc_h,
                                         int* size_init);

  double derivativePivot(double an, double bn, double cn, double ad, double bd,
                         double cd);  // calculate derivative of joint-angle w.r.t elbow-angle
  double jointAnglePivot(double an, double bn, double cn, double ad, double bd, double cd,
                         double gc);  // calculate joint-angle with elbow-angle set to lower_limit

  double derivativeHinge(double a, double b, double c, double gc);
  double jointAngleHinge(double a, double b, double c, double gc);

  bool operator<(const InvKinElbowInterval& rhs) const;  // compares lower_limit

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
  static double LIMBS[4];
  static InvKinJoints LOWER_JOINT_LIMITS;
  static InvKinJoints UPPER_JOINT_LIMITS;
  static bool INITIALIZED;

  // methods:

  // Compute FK:
  static InvKinMsg forwardKinematics(InvKinXCart* cart_pose, const InvKinJoints& joint_angles);

  // Compute IK:
  static InvKinMsg inverseKinematics(InvKinJoints* joint_angles, const InvKinXCart& cart_pose);
  // Compute IK and return helper matrices
  static InvKinMsg inverseKinematics(InvKinJoints* joint_angles, const InvKinXCart& cart_pose, Eigen::Matrix3d* as,
                                     Eigen::Matrix3d* bs, Eigen::Matrix3d* cs, Eigen::Matrix3d* aw, Eigen::Matrix3d* bw,
                                     Eigen::Matrix3d* cw, bool check_limits = false);

  // Initialize:
  bool initialize(const std::vector<double>& joint_distances, const std::vector<double>& lower_joint_limits,
                  const std::vector<double>& upper_joint_limits);

  // Compute feasible intervals in nullspace
  static InvKinMsg computeFeasibleIntervals(InvKinElbowInterval feasible_intervals[], const InvKinXCart& cart_pose,
                                            const Eigen::Matrix3d& as, const Eigen::Matrix3d& bs,
                                            const Eigen::Matrix3d& cs, const Eigen::Matrix3d& aw,
                                            const Eigen::Matrix3d& bw, const Eigen::Matrix3d& cw, int* n);

  // get closest solution to seed-state using optimization defined in optimize()
  static InvKinMsg getClosestPositionIK(InvKinJoints sol[], int* index_sol, const InvKinJoints& seed_state,
                                        const InvKinXCart& seed_state_x, InvKinXCart* cart_pose, const int configs[],
                                        int n_configs, InvKinMsg (*optimize)(InvKinElbowInterval[], int,
                                                                             const InvKinXCart&, InvKinXCart*));

  // redundancy resolution using exponential function
  static InvKinMsg redundancyResolutionExp(InvKinElbowInterval feasible_intervals[], int n,
                                           const InvKinXCart& seed_state, InvKinXCart* cart_pose);

  // wrapper-methods for KinematicsBase-Plugin
  // redundancy resolution using e-function and hard coded config in ik_pose.config
  static InvKinMsg getIKefuncFixedConfig(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution);
  // redundancy resolution using fixed nsparam
  static InvKinMsg getIKfixedNs(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution);
  // redundancy resolution using e-function and hard coded config and nsparam in ik_pose
  static InvKinMsg getIKefuncFixedConfigFixedNs(InvKinXCart ik_pose, InvKinJoints seed_state,
                                                std::vector<double>* solution);
  // redundancy-resolution using e-function
  static InvKinMsg getIKefunc(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution);

  static void determineClosestConfigs(int configs[], int* counter, const InvKinJoints& joint_angles);

private:
};

std::ostream& operator<<(std::ostream& stream, InvKinMsg const& val);

#endif /* INCLUDE_INVKINLIBRARY_H_ */
