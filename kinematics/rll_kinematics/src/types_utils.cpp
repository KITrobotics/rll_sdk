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

#include <iterator>

#include <rll_kinematics/types_utils.h>

const double RLLKinematicsBase::ZERO_ROUNDING_TOL = 1E-07;

const char* RLLKinMsg::message() const
{
  switch (value_)
  {
    case SUCCESS:
      return "SUCCESS";
    case CLOSE_TO_SINGULARITY:
      return "CLOSE_TO_SINGULARITY";
    case SINGULARITY:
      return "SINGULARITY";
    case INVALID_INPUT:
      return "INVALID_INPUT";
    case JOINT_LIMIT_VIOLATED:
      return "JOINT_LIMIT_VIOLATED";
    case TARGET_TOO_FAR:
      return "TARGET_TOO_FAR";
    case TARGET_TOO_CLOSE:
      return "TARGET_TOO_CLOSE";
    case NO_SOLUTION_FOR_ARMANGLE:
      return "NO_SOLUTION_FOR_ARMANGLE";
    case GENERAL_ERROR:
      return "GENERAL_ERROR";
    case NOT_SET:
      return "NOT_SET";

    default:
      return "UNKNOWN STATUS CODE";
  }
}

RLLKinJoints::RLLKinJoints(const std::initializer_list<double>& il)
{
  copyToJoints(il);
}

RLLKinJoints::RLLKinJoints(const std::vector<double>& v)
{
  setJoints(v);
}

void RLLKinJoints::setJoints(const std::vector<double>& v)
{
  copyToJoints(v);
}

void RLLKinJoints::getJoints(std::vector<double>* v)
{
  v->assign(joints_.begin(), joints_.end());
}

bool RLLKinJoints::jointLimitsViolated(const RLLKinJoints& lower_joint_limits,
                                       const RLLKinJoints& upper_joint_limits) const
{
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (kSmallerThan(joints_[i], lower_joint_limits(i)) || kGreaterThan(joints_[i], upper_joint_limits(i)))
    {
      return true;
    }
  }

  return false;
}

std::ostream& operator<<(std::ostream& out, const RLLKinJoints& j)
{
  std::copy(j.begin(), j.end(), std::ostream_iterator<double>(std::cout, " "));

  return out;
}

RLLKinFrame::RLLKinFrame(const double d, const double theta, const double a, const double alpha)
{
  double ca, sa;

  // alpha usually has these values
  if (kIsEqual(alpha, -M_PI / 2.0))
  {
    ca = 0.0;
    sa = -1.0;
  }
  else if (kIsEqual(alpha, 0.0))
  {
    ca = 1.0;
    sa = 0.0;
  }
  else if (kIsEqual(alpha, M_PI / 2.0))
  {
    ca = 0.0;
    sa = 1.0;
  }
  else
  {
    ca = cos(alpha);
    sa = sin(alpha);
  }

  double ct = cos(theta);
  double st = sin(theta);

  ori_ << ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0.0, sa, ca;

  pos_ << a * ct, a * st, d;
}

void RLLKinFrame::setQuaternion(const double w, const double x, const double y, const double z)
{
  ori_ = static_cast<Eigen::Matrix3d>(Eigen::Quaterniond(w, x, y, z));
}

void RLLKinFrame::setRPY(const double roll, const double pitch, const double yaw)
{
  ori_ = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void RLLKinFrame::getQuaternion(double* w, double* x, double* y, double* z) const
{
  Eigen::Quaterniond q = static_cast<Eigen::Quaterniond>(ori_);
  *w = q.w();
  *x = q.x();
  *y = q.y();
  *z = q.z();
}

void RLLKinFrame::setPosition(const double x, const double y, const double z)
{
  pos_ << x, y, z;
}

void RLLKinFrame::setPosition(const uint8_t index, const double value)
{
  pos_[index] = value;
}

RLLKinFrame RLLKinFrame::operator*(const RLLKinFrame& t) const
{
  RLLKinFrame result;
  result.ori_.noalias() = this->ori_ * t.ori_;
  result.pos_.noalias() = this->ori_ * t.pos_ + this->pos_;

  return result;
}

RLLKinFrame& RLLKinFrame::operator=(const RLLKinFrame& rhs)
{
  this->ori_.noalias() = rhs.ori_;
  this->pos_.noalias() = rhs.pos_;

  return *this;
}

void RLLKinGlobalConfig::set(const RLLKinJoints& joint_angles)
{
  // The global configuration is determined by the sign of the second, fourth and sixth joint angle.
  // For example, a configuration with value 6 = 110 means that the second axis is positive and the fourth and sixth
  // ones are negative
  config_ = (static_cast<uint8_t>(joint_angles(1) < -RLLKinematicsBase::ZERO_ROUNDING_TOL) << 0) |
            (static_cast<uint8_t>(joint_angles(3) < -RLLKinematicsBase::ZERO_ROUNDING_TOL) << 1) |
            (static_cast<uint8_t>(joint_angles(5) < -RLLKinematicsBase::ZERO_ROUNDING_TOL) << 2);
}

double RLLKinGlobalConfig::gc2() const
{
  return configAtJoint(0);
}

double RLLKinGlobalConfig::gc4() const
{
  return configAtJoint(1);
}

double RLLKinGlobalConfig::gc6() const
{
  return configAtJoint(2);
}

double RLLKinGlobalConfig::configAtJoint(const uint8_t bit) const
{
  if ((config_ & (1 << bit)) == 0)
  {
    return 1.0;
  }

  return -1.0;
}

uint8_t RLLKinGlobalConfig::indexGC4() const
{
  // positive angle is mapped to index 0, negative to 1
  // useful for accessing arrays depending on config
  if (gc4() > 0)
  {
    return 0;
  }

  return 1;
}

Eigen::Matrix3d RLLKinematicsBase::crossMatrix(const Eigen::Vector3d& vec) const
{
  Eigen::Matrix3d result;
  result << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;

  return result;
}

bool RLLKinematicsBase::kZero(const double f) const
{
  return fabs(f) <= ZERO_ROUNDING_TOL;
}

bool RLLKinematicsBase::kIsEqual(const double lhs, const double rhs) const
{
  return kZero(lhs - rhs);
}

bool RLLKinematicsBase::kGreaterZero(const double f) const
{
  return f >= -ZERO_ROUNDING_TOL;
}

bool RLLKinematicsBase::kGreaterThan(double lhs, double rhs) const
{
  return lhs > rhs + ZERO_ROUNDING_TOL;
}

bool RLLKinematicsBase::kSmallerThan(double lhs, double rhs) const
{
  return lhs < rhs - ZERO_ROUNDING_TOL;
}

double RLLKinematicsBase::kAcos(const double f) const
{
  if (f <= -1.0)
  {
    assert(kZero(f + 1.0));

    return M_PI;
  }

  if (f >= 1.0)
  {
    assert(kZero(f - 1.0));

    return 0.0;
  }

  return acos(f);
}

double RLLKinematicsBase::kSqrt(const double f) const
{
  assert(kGreaterZero(f));

  if (f <= 0.0)
  {
    return 0.0;
  }

  return sqrt(f);
}

double RLLKinematicsBase::kSign(const double f) const
{
  if (f > 0.0)
  {
    return 1.0;
  }

  if (f < 0.0)
  {
    return -1.0;
  }

  return 0.0;
}

double RLLKinematicsBase::mapAngleInPiRange(double angle) const
{
  // map to [-M_PI, M_PI] range

  while (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }

  while (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }

  return angle;
}
