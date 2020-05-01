/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_KINEMATICS_TYPES_UTILS_H
#define RLL_KINEMATICS_TYPES_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/container/static_vector.hpp>
#include <cmath>
#include <iostream>
#include <vector>

#define RLL_NUM_JOINTS 7

class RLLKinMsg
{
public:
  enum Msg : uint8_t
  {
    SUCCESS = 0,

    // warnings
    WARNINGS_BEGIN = 1,
    CLOSE_TO_SINGULARITY = WARNINGS_BEGIN,
    SINGULARITY,

    // errors
    ERRORS_BEGIN = 64,
    INVALID_INPUT = ERRORS_BEGIN,
    JOINT_LIMIT_VIOLATED,
    TARGET_TOO_FAR,
    TARGET_TOO_CLOSE,
    NO_SOLUTION_FOR_ARMANGLE,
    GENERAL_ERROR,

    NOT_SET = 255
  };

  RLLKinMsg() = default;

  RLLKinMsg(Msg msg) : value_(msg)  // NOLINT google-explicit-constructor
  {
  }

  Msg val() const
  {
    return value_;
  }

  bool error() const
  {
    return value_ >= ERRORS_BEGIN;
  }

  bool success() const
  {
    // yellow means green, like with traffic lights :)
    return value_ < ERRORS_BEGIN;
  }

  bool set() const
  {
    return value_ != NOT_SET;
  }

  const char* message() const;

private:
  Msg value_{ NOT_SET };
};

class RLLKinematicsBase
{
public:
  RLLKinematicsBase() = default;

  static const double ZERO_ROUNDING_TOL;

protected:
  Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& vec) const;

  bool kZero(double f) const;
  bool kGreaterZero(double f) const;
  double kAcos(double f) const;
  double kSqrt(double f) const;
  double kSign(double f) const;

  double mapAngleInPiRange(double angle) const;
};

class RLLKinJoints : public RLLKinematicsBase
{
public:
  RLLKinJoints() = default;
  RLLKinJoints(const std::initializer_list<double>& il);  // NOLINT google-explicit-constructor
  RLLKinJoints(const std::vector<double>& v);             // NOLINT google-explicit-constructor

  void setJoints(const std::vector<double>& v);

  double& operator[](int i)
  {
    return joints_[i];
  }

  double operator()(int i) const
  {
    return joints_[i];
  }

  std::array<double, RLL_NUM_JOINTS>::const_iterator begin() const
  {
    return joints_.begin();
  }

  std::array<double, RLL_NUM_JOINTS>::const_iterator end() const
  {
    return joints_.end();
  }

  void getJoints(std::vector<double>* v);

  bool jointLimitsViolated(const RLLKinJoints& lower_joint_limits, const RLLKinJoints& upper_joint_limits) const;

private:
  template <class container>
  void copyToJoints(const container& c);

  std::array<double, RLL_NUM_JOINTS> joints_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
};

class RLLKinFrame
{
public:
  RLLKinFrame() = default;
  RLLKinFrame(double d, double theta, double a, double alpha);  // construct frame from DH-parameters

  Eigen::Matrix3d const& ori() const
  {
    return ori_;
  }
  Eigen::Vector3d const& pos() const
  {
    return pos_;
  }
  void getQuaternion(double* w, double* x, double* y, double* z) const;
  void setQuaternion(double w, double x, double y, double z);
  void setRPY(double roll, double pitch, double yaw);

  void setPosition(double x, double y, double z);
  void setPosition(uint8_t index, double value);

  RLLKinFrame operator*(const RLLKinFrame& t) const;

private:
  Eigen::Matrix3d ori_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
};

class RLLKinGlobalConfig
{
public:
  RLLKinGlobalConfig() = default;

  explicit RLLKinGlobalConfig(const uint8_t config)
  {
    config_ = config;
  }

  explicit RLLKinGlobalConfig(const RLLKinJoints& joint_angles)
  {
    set(joint_angles);
  }

  void set(const RLLKinJoints& joint_angles);

  void set(const uint8_t config)
  {
    config_ = config;
  }

  double gc2() const;
  double gc4() const;
  double gc6() const;

  uint8_t indexGC4() const;

  uint8_t val() const
  {
    return config_;
  }

  bool operator==(const RLLKinGlobalConfig& n2)
  {
    return config_ == n2.config_;
  }

private:
  uint8_t config_ = 0;

  double configAtJoint(uint8_t bit) const;
};

using RLLKinGlobalConfigs = boost::container::static_vector<RLLKinGlobalConfig, 8>;

struct RLLKinPoseConfig
{
  RLLKinFrame pose;
  double arm_angle = 0.0;
  RLLKinGlobalConfig config;
};

template <class container>
void RLLKinJoints::copyToJoints(const container& c)
{
  size_t num_joints = std::min<size_t>(c.size(), RLL_NUM_JOINTS);

  std::copy_n(c.begin(), num_joints, joints_.begin());
}

#endif  // RLL_KINEMATICS_TYPES_UTILS_H
