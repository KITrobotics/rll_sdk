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

#include "inverse_kinematics_library.h"

// InvKin properties
double InvKin::limbs_[4];
InvKinJoints InvKin::lower_joint_limits_;
InvKinJoints InvKin::upper_joint_limits_;
bool InvKin::initialized_ = false;

// Function to generate Cross product matrix:
inline Matrix3d crossMatrix(Vector3d& mat)
{
  Matrix3d result;
  result << 0, -mat(2), mat(1), mat(2), 0, -mat(0), -mat(1), mat(0), 0;
  return result;
}

bool InvKinJoints::setJoints(const std::vector<double>& v)
{
  for (size_t i = 0; i < NR_JOINTS; i++)
  {
    j[i] = v[i];
  }

  return true;
}

double& InvKinJoints::operator[](int i)
{
  return this->j[i];
}

InvKinFrame::InvKinFrame(double d, double theta, double a, double alpha)
{
  double ca = cos(alpha);
  double sa = sin(alpha);
  double ct = cos(theta);
  double st = sin(theta);

  ori << ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca;

  pos << a * ct, a * st, d;
}

void InvKinFrame::setQuaternion(double w, double x, double y, double z)
{
  ori = (Matrix3d)Quaterniond(w, x, y, z);
}

void InvKinFrame::getQuaternion(double& w, double& x, double& y, double& z) const
{
  Quaterniond q = (Quaterniond)ori;
  w = q.w();
  x = q.x();
  y = q.y();
  z = q.z();
}

void InvKinFrame::setPosition(double x, double y, double z)
{
  pos << x, y, z;
}

InvKinFrame InvKinFrame::operator*(InvKinFrame T)
{
  InvKinFrame result;
  result.ori = this->ori * T.ori;
  result.pos = this->ori * T.pos + this->pos;
  return result;
}

bool InvKinElbowInterval::operator<(InvKinElbowInterval b) const
{
  // non-initialized intervals are bigger
  if (!init)
  {
    return false;
  }
  if (init && !b.init)
  {
    return true;
  }
  if (lower_limit < b.lower_limit)
  {
    return true;
  }
  return false;
}

void InvKinElbowInterval::quickSortLower(InvKinElbowInterval intervals[], int left, int right)
{
  int i = left, j = right;
  InvKinElbowInterval tmp;
  InvKinElbowInterval pivot = intervals[(left + right) / 2];

  // partition
  while (i <= j)
  {
    while (intervals[i] < pivot)
      i++;
    while (pivot < intervals[j])
      j--;
    if (i <= j)
    {
      tmp = intervals[i];
      intervals[i] = intervals[j];
      intervals[j] = tmp;
      i++;
      j--;
    }
  }

  // recursion
  if (left < j)
  {
    quickSortLower(intervals, left, j);
  }
  if (i < right)
  {
    quickSortLower(intervals, i, right);
  }
}

void InvKinElbowInterval::mergeSortedIntervals(InvKinElbowInterval intervals[], int n)
{
  std::vector<InvKinElbowInterval> stack;

  stack.push_back(intervals[0]);

  for (size_t i = 1; i < n; i++)
  {
    if (!intervals[i].init)
      break;

    InvKinElbowInterval back = stack.back();

    if (back.upper_limit < intervals[i].lower_limit)
    {
      stack.push_back(intervals[i]);
    }
    else if (back.upper_limit < intervals[i].upper_limit)
    {
      back.upper_limit = intervals[i].upper_limit;
      stack.pop_back();
      stack.push_back(back);
    }

    intervals[i].reset();
  }

  size_t i = 0;

  std::for_each(stack.begin(), stack.end(), [intervals, &i](InvKinElbowInterval e) {
    intervals[i] = e;
    i++;
  });
}

void InvKinElbowInterval::determineBlockedIntervalsPivot(InvKinElbowInterval interval_limits[],
                                                         InvKinElbowInterval blocked_intervals[], double& an,
                                                         double& bn, double& cn, double& ad, double& bd, double& cd,
                                                         int size)
{
  // classification if blocked or feasible interval depends on derivative of joint angle w.r.t elbow-angle (sign)

  for (size_t j = 0; j < size; j++)
  {
    if (!interval_limits[j].init)
    {
      break;  // no further blocked intervals
    }

    if (j != (size - 1) && interval_limits[j + 1].init)
    {
      if (signbit(interval_limits[j].add) == signbit(interval_limits[j].derivativePivot(an, bn, cn, ad, bd, cd)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

        continue;
      }
      else
      {
        if (signbit(interval_limits[j + 1].add) !=
            signbit(interval_limits[j + 1].derivativePivot(an, bn, cn, ad, bd, cd)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

          continue;
        }
      }
    }
    else
    {
      if (signbit(interval_limits[j].add) == signbit(interval_limits[j].derivativePivot(an, bn, cn, ad, bd, cd)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
        blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

        break;
      }
      else
      {
        if (signbit(interval_limits[0].add) != signbit(interval_limits[0].derivativePivot(an, bn, cn, ad, bd, cd)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
          blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

          break;
        }
      }
    }
  }
}

void InvKinElbowInterval::determineBlockedIntervalsHinge(InvKinElbowInterval interval_limits[],
                                                         InvKinElbowInterval blocked_intervals[], double& a, double& b,
                                                         double& c, double& gc_h, int size)
{
  // classification if blocked or feasible interval depends on derivative of joint angle w.r.t elbow-angle (sign)

  for (size_t j = 0; j < size; j++)
  {
    if (!interval_limits[j].init)
    {
      break;  // no further blocked intervals
    }

    if (j != (size - 1) && interval_limits[j + 1].init)
    {
      if (signbit(interval_limits[j].add) == signbit(interval_limits[j].derivativeHinge(a, b, c, gc_h)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

        continue;
      }
      else
      {
        if (signbit(interval_limits[j + 1].add) != signbit(interval_limits[j + 1].derivativeHinge(a, b, c, gc_h)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

          continue;
        }
      }
    }
    else
    {
      if (signbit(interval_limits[j].add) == signbit(interval_limits[j].derivativeHinge(a, b, c, gc_h)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
        blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

        break;
      }
      else
      {
        if (signbit(interval_limits[0].add) != signbit(interval_limits[0].derivativeHinge(a, b, c, gc_h)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
          blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

          break;
        }
      }
    }
  }
}

void InvKinElbowInterval::mapLimitsToElbowAnglePivot(InvKinElbowInterval interval_limits[], double& lower_joint_limit,
                                                     double& upper_joint_limit, double& an, double& bn, double& cn,
                                                     double& ad, double& bd, double& cd, double& gc_p, int& size_init)
{
  double ap, bp, cp;

  // joint angle limits mapped to elbow angle
  ap = gc_p * ((cd - bd) * tan(lower_joint_limit) + (bn - cn));
  bp = 2 * gc_p * (ad * tan(lower_joint_limit) - an);
  cp = gc_p * ((bd + cd) * tan(lower_joint_limit) - (bn + cn));

  if (bp * bp >= 4 * ap * cp)  // map lower joint limits to elbow angle
  {
    interval_limits[0].setLimits(2 * atan((-bp - sqrt(bp * bp - 4 * ap * cp)) / (2 * ap)), 0.0);
    interval_limits[1].setLimits(2 * atan((-bp + sqrt(bp * bp - 4 * ap * cp)) / (2 * ap)), 0.0);
    interval_limits[0].setAdd(lower_joint_limit);
    interval_limits[1].setAdd(lower_joint_limit);
  }

  ap = gc_p * ((cd - bd) * tan(upper_joint_limit) + (bn - cn));
  bp = 2 * gc_p * (ad * tan(upper_joint_limit) - an);
  cp = gc_p * ((bd + cd) * tan(upper_joint_limit) - (bn + cn));

  if (bp * bp >= 4 * ap * cp)  // map upper joint limits to elbow angle
  {
    interval_limits[2].setLimits(2 * atan((-bp - sqrt(bp * bp - 4 * ap * cp)) / (2 * ap)), 0.0);
    interval_limits[3].setLimits(2 * atan((-bp + sqrt(bp * bp - 4 * ap * cp)) / (2 * ap)), 0.0);
    interval_limits[2].setAdd(upper_joint_limit);
    interval_limits[3].setAdd(upper_joint_limit);
  }

  size_init = 0;
  for (size_t j = 0; j < 4; j++)
  {  // check if calculated elbow-limit is matching to limit in joint-space
    if (abs(interval_limits[j].add - interval_limits[j].jointAnglePivot(an, bn, cn, ad, bd, cd, gc_p)) >= 1E-06)
      interval_limits[j].reset();
    if (interval_limits[j].init)
      size_init++;
  }
}

void InvKinElbowInterval::mapLimitsToElbowAngleHinge(InvKinElbowInterval interval_limits[], double& lower_joint_limit,
                                                     double& upper_joint_limit, double& a, double& b, double& c,
                                                     double& gc_h, int& size_init)
{
  // joint angle limits mapped to elbow angle
  if (a * a + b * b - pow(c - cos(lower_joint_limit), 2) >= 0.0)
  {
    interval_limits[0].setLimits(
        2 * atan((a - sqrt(a * a + b * b - pow(c - cos(lower_joint_limit), 2))) / (cos(lower_joint_limit) + b - c)),
        0.0);
    interval_limits[1].setLimits(
        2 * atan((a + sqrt(a * a + b * b - pow(c - cos(lower_joint_limit), 2))) / (cos(lower_joint_limit) + b - c)),
        0.0);
    interval_limits[0].setAdd(lower_joint_limit);
    interval_limits[1].setAdd(lower_joint_limit);
  }

  if (a * a + b * b - pow(c - cos(upper_joint_limit), 2) >= 0.0)
  {
    interval_limits[2].setLimits(
        2 * atan((a - sqrt(a * a + b * b - pow(c - cos(upper_joint_limit), 2))) / (cos(upper_joint_limit) + b - c)),
        0.0);
    interval_limits[3].setLimits(
        2 * atan((a + sqrt(a * a + b * b - pow(c - cos(upper_joint_limit), 2))) / (cos(upper_joint_limit) + b - c)),
        0.0);
    interval_limits[2].setAdd(upper_joint_limit);
    interval_limits[3].setAdd(upper_joint_limit);
  }

  size_init = 0;
  for (size_t j = 0; j < 4; j++)
  {  // check if calculated elbow-limit is matching to limit in joint-space
    if (abs(interval_limits[j].add - interval_limits[j].jointAngleHinge(a, b, c, gc_h)) >= 1E-06)
      interval_limits[j].reset();
    if (interval_limits[j].init)
      size_init++;
  }
}

inline double InvKinElbowInterval::derivativePivot(const double& an, const double& bn, const double& cn,
                                                   const double& ad, const double& bd, const double& cd)
{
  // helper coefficients
  double at = (cn * bd - bn * cd);  // not linear in gc (failure in paper)
  double bt = (an * cd - cn * ad);
  double ct = (an * bd - bn * ad);

  double u = (an * sin(lower_limit) + bn * cos(lower_limit) + cn);
  double v = (ad * sin(lower_limit) + bd * cos(lower_limit) + cd);

  // derivative
  return (at * sin(lower_limit) + bt * cos(lower_limit) + ct) / (u * u + v * v);
}

inline double InvKinElbowInterval::jointAnglePivot(const double& an, const double& bn, const double& cn,
                                                   const double& ad, const double& bd, const double& cd,
                                                   const double& GC)
{
  return atan2(GC * (an * sin(lower_limit) + bn * cos(lower_limit) + cn),
               GC * (ad * sin(lower_limit) + bd * cos(lower_limit) + cd));
}

inline double InvKinElbowInterval::derivativeHinge(const double& a, const double& b, const double& c, const double& GC)
{
  return -GC * ((a * cos(lower_limit) - b * sin(lower_limit)) /
                (abs(sin(add))));  // division by zero impossible, function is only called in joint limit != 0 or Pi.
}

inline double InvKinElbowInterval::jointAngleHinge(const double& a, const double& b, const double& c, const double& GC)
{
  return GC * acos(a * sin(lower_limit) + b * cos(lower_limit) + c);
}

InvKinMsg InvKin::forwardKinematics(InvKinXCart& xCartPose, InvKinJoints& jointAngles)
{
  InvKinMsg result = InvKin_OK;

  InvKinFrame mbs;  // shoulder pose
  InvKinFrame mbe;  // elbow pose
  InvKinFrame mbw;  // wrist pose
  InvKinFrame mbf;  // flange pose

  mbs = InvKinFrame(limbs_[0], jointAngles[0], 0.0, -M_PI / 2.0) * InvKinFrame(0.0, jointAngles[1], 0.0, M_PI / 2.0);
  mbe = mbs * InvKinFrame(limbs_[1], jointAngles[2], 0.0, M_PI / 2.0) *
        InvKinFrame(0.0, jointAngles[3], 0.0, -M_PI / 2.0);
  mbw = mbe * InvKinFrame(limbs_[2], jointAngles[4], 0.0, -M_PI / 2.0) *
        InvKinFrame(0.0, jointAngles[5], 0.0, M_PI / 2.0);
  mbf = mbw * InvKinFrame(limbs_[3], jointAngles[6], 0.0, 0.0);

  xCartPose.pose = mbf;

  // determine configuration
  xCartPose.config = (double)(jointAngles[1] < 0) + (double)(jointAngles[3] < 0) * 2 + (double)(jointAngles[5] < 0) * 4;

  // determine reference plane and null-space-parameter (elbow-redundancy)

  Vector3d xsw = mbw.pos - mbs.pos;  // Vector shoulder to wrist
  Vector3d xsw_n = xsw;              // normalized
  xsw_n.normalize();
  double lsw = xsw.norm();

  double joint_angle_1_v;  // virtual base joint angle
  if (sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)) < 1E-06)
  {
    joint_angle_1_v = 0.0;
  }
  else
  {
    joint_angle_1_v = atan2(xsw(1), xsw(0));
  }

  // determine virtual shoulder joint angle, depending on configuration
  double joint_angle_2_v = acos(((limbs_[1] * limbs_[1]) + (lsw * lsw) - (limbs_[2] * limbs_[2])) /
                                (2 * limbs_[1] * lsw));  // virtual shoulder joint angle, only intermediate step

  if ((xCartPose.config & 2) == 0)
  {  // elbow joint angle zero/positive if second bit is not set
    joint_angle_2_v = atan2(sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)), xsw(2)) + joint_angle_2_v;
  }
  else
  {
    joint_angle_2_v = atan2(sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)), xsw(2)) - joint_angle_2_v;
  }

  InvKinFrame mbs_v;  // virtual elbow pose
  mbs_v =
      InvKinFrame(limbs_[0], joint_angle_1_v, 0.0, -M_PI / 2.0) * InvKinFrame(0.0, joint_angle_2_v, 0.0, M_PI / 2.0);
  Vector3d xse_v;  // virtual shoulder to elbow vector
  xse_v << 0.0, 0.0, limbs_[1];
  Vector3d xseb_v;  // virtual shoulder to elbow vector in base coordinates
  xseb_v = mbs_v.ori * xse_v;
  Vector3d xseb_n_v = xseb_v;  // normalized
  xseb_n_v.normalize();

  Vector3d v_sew_v;  // virtual reference plane (shoulder, elbow, wrist) normal vector
  v_sew_v = xseb_n_v.cross(xsw_n);
  v_sew_v.normalize();

  Vector3d v_sew;   // real reference plane (shoulder, elbow, wrist) normal vector
  Vector3d xseb_n;  // real normalized shoulder to elbow vector in base coordinates
  xseb_n = mbe.pos - mbs.pos;
  xseb_n.normalize();  // TODO: nomalize necessary in this context??
  v_sew = xseb_n.cross(xsw_n);
  v_sew.normalize();

  double psi;
  psi = (v_sew_v.cross(v_sew)).transpose() * xsw;
  double nsparam_temp;
  nsparam_temp = v_sew_v.transpose() * v_sew;

  if (psi >= 0)
  {
    if (nsparam_temp >= 1.0)
    {  // nsparam_temp could be greater than 1.0 due to numerical deviations
      xCartPose.nsparam = 0.0;
    }
    else if (nsparam_temp <= -1.0)
    {
      xCartPose.nsparam = M_PI;
    }
    else
    {
      xCartPose.nsparam = acos(nsparam_temp);
    }
  }
  else
  {
    if (nsparam_temp >= 1.0)
    {
      xCartPose.nsparam = -0.0;
    }
    else if (nsparam_temp <= -1.0)
    {
      xCartPose.nsparam = -M_PI;
    }
    else
    {
      xCartPose.nsparam = -acos(nsparam_temp);
    }
  }

  return result;
}

InvKinMsg InvKin::inverseKinematics(InvKinJoints& jointAngles, InvKinXCart& xCartPose)
{
  Matrix3d as, bs, cs;  // helper matrix As, Bs, Cs
  Matrix3d aw, bw, cw;  // helper matrix Aw, Bw, Cw

  return InvKin::inverseKinematics(jointAngles, xCartPose, as, bs, cs, aw, bw, cw, true);
}

InvKinMsg InvKin::inverseKinematics(InvKinJoints& jointAngles, InvKinXCart& xCartPose, Matrix3d& As, Matrix3d& Bs,
                                    Matrix3d& Cs, Matrix3d& Aw, Matrix3d& Bw, Matrix3d& Cw, bool check_limits)
{
  InvKinMsg result = InvKin_OK;

  InvKinFrame mfw;  // wrist pose in flange coordinates
  mfw.pos[2] = -limbs_[3];

  Vector3d xw = (xCartPose.pose * mfw).pos;  // wrist position in base coordinates
  Vector3d xs(0, 0, limbs_[0]);              // shoulder pos

  Vector3d xsw = xw - xs;  // vector from shoulder to wrist
  double lsw = xsw.norm();

  // check if target is too close/far
  if (lsw > limbs_[1] + limbs_[2])
  {
    result = (InvKinMsg)(InvKin_ERROR | InvKin_TARGET_TOO_FAR);
    return result;
  }

  if (lsw < std::abs(limbs_[1] - limbs_[2]))
  {
    result = (InvKinMsg)(InvKin_ERROR | InvKin_TARGET_TOO_CLOSE);
    return result;
  }

  Vector3d xsw_n = xsw;  // normalized shoulder to wrist vector
  xsw_n.normalize();
  Matrix3d xsw_n_cross = crossMatrix(xsw_n);  // cross product matrix of xsw_n

  // determine virtual elbow / upper arm pose

  double joint_angle_1_v;  // virtual base joint angle

  if (sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)) < 1E-06)
  {
    joint_angle_1_v = 0.0;
  }
  else
  {
    joint_angle_1_v = atan2(xsw(1), xsw(0));
  }

  double joint_angle_2_v = acos(((limbs_[1] * limbs_[1]) + (lsw * lsw) - (limbs_[2] * limbs_[2])) /
                                (2 * limbs_[1] * lsw));  // virtual shoulder joint angle, only intermediate step

  // determine elbow angle using the law of cosines and virtual shoulder joint angle, depending on configuration
  if ((xCartPose.config & 2) == 0)
  {  // elbow joint angle zero/positive if second bit is not set
    jointAngles[3] =
        acos(((lsw * lsw) - (limbs_[1] * limbs_[1]) - (limbs_[2] * limbs_[2])) / (2 * limbs_[1] * limbs_[2]));
    joint_angle_2_v = atan2(sqrt((xsw(0) * xsw(0)) + (xsw(1) * xsw(1))), xsw(2)) + joint_angle_2_v;
  }
  else
  {
    jointAngles[3] =
        -acos(((lsw * lsw) - (limbs_[1] * limbs_[1]) - (limbs_[2] * limbs_[2])) / (2 * limbs_[1] * limbs_[2]));
    joint_angle_2_v = atan2(sqrt((xsw(0) * xsw(0)) + (xsw(1) * xsw(1))), xsw(2)) - joint_angle_2_v;
  }

  InvKinFrame mbu_v;  // virtual upper arm pose
  mbu_v = InvKinFrame(limbs_[0], joint_angle_1_v, 0.0, -M_PI / 2.0) *
          InvKinFrame(0.0, joint_angle_2_v, 0.0, M_PI / 2.0) * InvKinFrame(limbs_[1], 0.0, 0.0, M_PI / 2.0);

  // determine real pose as rotation of virtual pose

  // helper matrix As, Bs, Cs to rotate mbu_v
  As = xsw_n_cross * mbu_v.ori;
  Bs = -xsw_n_cross * As;
  Cs = (xsw_n * xsw_n.transpose()) * mbu_v.ori;

  // real joint angles, depending on configuration:

  if ((xCartPose.config & 1) == 0)
  {  // shoulder joint angle zero/positive if first bit is not set
    jointAngles[0] = atan2((As(1, 1) * sin(xCartPose.nsparam) + Bs(1, 1) * cos(xCartPose.nsparam) + Cs(1, 1)),
                           (As(0, 1) * sin(xCartPose.nsparam) + Bs(0, 1) * cos(xCartPose.nsparam) + Cs(0, 1)));

    jointAngles[1] = acos(As(2, 1) * sin(xCartPose.nsparam) + Bs(2, 1) * cos(xCartPose.nsparam) + Cs(2, 1));

    jointAngles[2] = atan2(-(As(2, 2) * sin(xCartPose.nsparam) + Bs(2, 2) * cos(xCartPose.nsparam) + Cs(2, 2)),
                           -(As(2, 0) * sin(xCartPose.nsparam) + Bs(2, 0) * cos(xCartPose.nsparam) + Cs(2, 0)));
  }
  else
  {
    jointAngles[0] = atan2(-(As(1, 1) * sin(xCartPose.nsparam) + Bs(1, 1) * cos(xCartPose.nsparam) + Cs(1, 1)),
                           -(As(0, 1) * sin(xCartPose.nsparam) + Bs(0, 1) * cos(xCartPose.nsparam) + Cs(0, 1)));

    jointAngles[1] = -acos(As(2, 1) * sin(xCartPose.nsparam) + Bs(2, 1) * cos(xCartPose.nsparam) + Cs(2, 1));

    jointAngles[2] = atan2((As(2, 2) * sin(xCartPose.nsparam) + Bs(2, 2) * cos(xCartPose.nsparam) + Cs(2, 2)),
                           (As(2, 0) * sin(xCartPose.nsparam) + Bs(2, 0) * cos(xCartPose.nsparam) + Cs(2, 0)));
  }

  InvKinFrame mue;  // elbow pose in upper arm coordinates, same as virtual elbow pose
  mue = InvKinFrame(0.0, jointAngles[3], 0.0, -M_PI / 2.0);

  // helper matrix Aw, Bw, Cw

  Aw = mue.ori.transpose() * As.transpose() * xCartPose.pose.ori;
  Bw = mue.ori.transpose() * Bs.transpose() * xCartPose.pose.ori;
  Cw = mue.ori.transpose() * Cs.transpose() * xCartPose.pose.ori;

  if ((xCartPose.config & 4) == 0)
  {  // wrist joint angle zero/positive if third bit is not set
    jointAngles[4] = atan2(Aw(1, 2) * sin(xCartPose.nsparam) + Bw(1, 2) * cos(xCartPose.nsparam) + Cw(1, 2),
                           Aw(0, 2) * sin(xCartPose.nsparam) + Bw(0, 2) * cos(xCartPose.nsparam) + Cw(0, 2));
    jointAngles[5] = acos(Aw(2, 2) * sin(xCartPose.nsparam) + Bw(2, 2) * cos(xCartPose.nsparam) + Cw(2, 2));
    jointAngles[6] = atan2((Aw(2, 1) * sin(xCartPose.nsparam) + Bw(2, 1) * cos(xCartPose.nsparam) + Cw(2, 1)),
                           -(Aw(2, 0) * sin(xCartPose.nsparam) + Bw(2, 0) * cos(xCartPose.nsparam) + Cw(2, 0)));
  }
  else
  {
    jointAngles[4] = atan2(-(Aw(1, 2) * sin(xCartPose.nsparam) + Bw(1, 2) * cos(xCartPose.nsparam) + Cw(1, 2)),
                           -(Aw(0, 2) * sin(xCartPose.nsparam) + Bw(0, 2) * cos(xCartPose.nsparam) + Cw(0, 2)));
    jointAngles[5] = -acos(Aw(2, 2) * sin(xCartPose.nsparam) + Bw(2, 2) * cos(xCartPose.nsparam) + Cw(2, 2));
    jointAngles[6] = atan2(-(Aw(2, 1) * sin(xCartPose.nsparam) + Bw(2, 1) * cos(xCartPose.nsparam) + Cw(2, 1)),
                           (Aw(2, 0) * sin(xCartPose.nsparam) + Bw(2, 0) * cos(xCartPose.nsparam) + Cw(2, 0)));
  }

  // check for joint limits and singularities depending on check_limits
  if (check_limits)
  {
    result = InvKin_OK;

    for (int j = 0; j < 7; j++)
    {
      if ((jointAngles[j] < lower_joint_limits_[j]) || (jointAngles[j] > upper_joint_limits_[j]))
      {
        result = (InvKinMsg)(result | InvKin_WARNING | InvKin_JOINTLIMIT);
      }
    }

    for (int j = 1; j <= 5; j += 2)
    {
      if (std::abs(jointAngles[j]) < 15.0 / 180.0 * M_PI)
      {
        result = (InvKinMsg)(result | InvKin_WARNING | InvKin_CLOSE_TO_SINGULARITY);
      }
    }

    Vector3d z;
    z << 0, 0, 1;

    double overhead = acos(xsw.dot(z) / lsw);
    if (overhead < 15.0 / 180.0 * M_PI || overhead > 165.0 / 180.0 * M_PI)
    {
      result = (InvKinMsg)(result | InvKin_WARNING | InvKin_CLOSE_TO_SINGULARITY);
    }
  }

  return result;
}

bool InvKin::initialize(const std::vector<double>& joint_distances, const std::vector<double>& lower_jointLimits,
                        const std::vector<double>& upper_jointLimits)
{
  this->limbs_[0] = joint_distances[0] + joint_distances[1];
  this->limbs_[1] = joint_distances[2] + joint_distances[3];
  this->limbs_[2] = joint_distances[4] + joint_distances[5];
  this->limbs_[3] = joint_distances[6] + joint_distances[7];

  this->lower_joint_limits_.setJoints(lower_jointLimits);
  this->upper_joint_limits_.setJoints(upper_jointLimits);

  initialized_ = true;

  return true;
}

InvKinMsg InvKin::computeFeasibleIntervals(InvKinElbowInterval feasible_intervals[], InvKinXCart& xCartPose,
                                           Matrix3d& As, Matrix3d& Bs, Matrix3d& Cs, Matrix3d& Aw, Matrix3d& Bw,
                                           Matrix3d& Cw, int& n)
{
  InvKinMsg result = InvKin_OK;
  double margin = 0.05;  // blocked margin around singular elbow-angle
  int counter;

  InvKinElbowInterval blocked_intervals[34];

  // ..._p: for pivot-type joints
  InvKinElbowInterval blocked_intervals_limit_p[4][4];  // blocked intervals due to joint limits
  InvKinElbowInterval blocked_intervals_sing_p[4];      // blocked intervals due to singularities
  InvKinElbowInterval blocked_intervals_p[4][5];        // blocked intervals for all 4 joints of pivot-type
  double psi_singular;

  double gc_p[4];              // Configuration(GC) - parameter
  double an[4], bn[4], cn[4];  // coefficients used to get generic shapes for different functions
  double ad[4], bd[4], cd[4];
  double at, bt, ct;

  // ..._h: for hinge-type joints
  InvKinElbowInterval blocked_intervals_limit_h[2][4];  // blocked intervals due to joint limits
  InvKinElbowInterval blocked_intervals_h[2][5];        // blocked intervals for all 2 joints of hinge-type
  double gc_h[2];                                       // Configuration(GC) - parameter
  double a[2], b[2], c[2];  // coefficients used to get generic shapes for different functions

  // initialize configuration-parameters
  if ((xCartPose.config & 1) == 0)
  {  // shoulder joint angle zero/positive if first bit is not set
    gc_p[0] = 1.0;
    gc_p[1] = 1.0;
    gc_h[0] = 1.0;
  }
  else
  {
    gc_p[0] = -1.0;
    gc_p[1] = -1.0;
    gc_h[0] = -1.0;
  }

  if ((xCartPose.config & 4) == 0)
  {  // wrist joint angle zero/positive if third bit is not set
    gc_p[2] = 1.0;
    gc_p[3] = 1.0;
    gc_h[1] = 1.0;
  }
  else
  {
    gc_p[2] = -1.0;
    gc_p[3] = -1.0;
    gc_h[1] = -1.0;
  }

  // initialize coefficients for all joints
  an[0] = As(1, 1);
  bn[0] = Bs(1, 1);
  cn[0] = Cs(1, 1);
  ad[0] = As(0, 1);
  bd[0] = Bs(0, 1);
  cd[0] = Cs(0, 1);

  an[1] = -As(2, 2);
  bn[1] = -Bs(2, 2);
  cn[1] = -Cs(2, 2);
  ad[1] = -As(2, 0);
  bd[1] = -Bs(2, 0);
  cd[1] = -Cs(2, 0);

  an[2] = Aw(1, 2);
  bn[2] = Bw(1, 2);
  cn[2] = Cw(1, 2);
  ad[2] = Aw(0, 2);
  bd[2] = Bw(0, 2);
  cd[2] = Cw(0, 2);

  an[3] = Aw(2, 1);
  bn[3] = Bw(2, 1);
  cn[3] = Cw(2, 1);
  ad[3] = -Aw(2, 0);
  bd[3] = -Bw(2, 0);
  cd[3] = -Cw(2, 0);

  a[0] = As(2, 1);
  b[0] = Bs(2, 1);
  c[0] = Cs(2, 1);
  a[1] = Aw(2, 2);
  b[1] = Bw(2, 2);
  c[1] = Cw(2, 2);

  ///////////////////
  // Pivot Joints //
  //////////////////
  for (size_t i = 0; i < 4; i++)
  {  // for all pivot-joints

    // determine singularity in joint angle
    at = (cn[i] * bd[i] - bn[i] * cd[i]);  // no gc-parameter in at, bt, ct (failure in paper)
    bt = (an[i] * cd[i] - cn[i] * ad[i]);
    ct = (an[i] * bd[i] - bn[i] * ad[i]);

    if (abs(at * at + bt * bt - ct * ct) <= 1E-06)
    {
      psi_singular = 2 * atan(at / (bt - ct));
      blocked_intervals_sing_p[i].setLimits(
          psi_singular - margin, psi_singular + margin);  // blocked interval due to singularity at psi_singular
    }

    // joint angle limits mapped to elbow angle
    // lower_limit (elbow angle) of InvKinElbowInterval is initialized with corresponding joint angle limit (upper and
    // lower)
    // add of InvKinElbowInterval is used here to remember the corresponding joint angle limit
    InvKinElbowInterval::mapLimitsToElbowAnglePivot(blocked_intervals_limit_p[i], lower_joint_limits_[2 * i],
                                                    upper_joint_limits_[2 * i], an[i], bn[i], cn[i], ad[i], bd[i],
                                                    cd[i], gc_p[i], counter);

    // sort the lower_limits of InvKinElbowInterval[] --> uninitialized InvKinElbowIntervals move to end
    InvKinElbowInterval::quickSortLower(blocked_intervals_limit_p[i], 0, 3);

    // determine blocked intervals due to joint limits: up to here only limits are mapped to elbow angle
    // --> classification in blocked or feasible intervals necessary
    if (counter > 0)
    {
      // classification if blocked or feasible interval depending on
      // derivative of joint angle w.r.t elbow-angle (sign)
      InvKinElbowInterval::determineBlockedIntervalsPivot(blocked_intervals_limit_p[i], blocked_intervals_p[i], an[i],
                                                          bn[i], cn[i], ad[i], bd[i], cd[i], 4);
    }
    else
    {
      if ((blocked_intervals_limit_p[i][0].jointAnglePivot(an[i], bn[i], cn[i], ad[i], bd[i], cd[i], gc_p[i]) >=
           upper_joint_limits_[2 * i]) ||
          (blocked_intervals_limit_p[i][0].jointAnglePivot(an[i], bn[i], cn[i], ad[i], bd[i], cd[i], gc_p[i]) <=
           lower_joint_limits_[2 * i]))
      {  // all angles blocked
        blocked_intervals_p[i][0].setLimits(-M_PI, M_PI);
      }
    }
  }

  ///////////////////
  // Hinge Joints //
  //////////////////
  for (size_t i = 0; i < 2; i++)
  {
    // joint angle limits mapped to elbow angle
    InvKinElbowInterval::mapLimitsToElbowAngleHinge(blocked_intervals_limit_h[i], lower_joint_limits_[4 * i + 1],
                                                    upper_joint_limits_[4 * i + 1], a[i], b[i], c[i], gc_h[i], counter);

    InvKinElbowInterval::quickSortLower(blocked_intervals_limit_h[i], 0, 3);

    if (counter > 0)
    {
      InvKinElbowInterval::determineBlockedIntervalsHinge(blocked_intervals_limit_h[i], blocked_intervals_h[i], a[i],
                                                          b[i], c[i], gc_h[i], 4);
    }
    else
    {
      if ((blocked_intervals_limit_h[i][0].jointAngleHinge(a[i], b[i], c[i], gc_h[i]) >=
           upper_joint_limits_[4 * i + 1]) ||
          (blocked_intervals_limit_h[i][0].jointAngleHinge(a[i], b[i], c[i], gc_h[i]) <=
           lower_joint_limits_[4 * i + 1]))
      {
        blocked_intervals_h[i][0].setLimits(-M_PI, M_PI);
      }
    }
  }
  ///////////////////

  // merge all intervals
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 5; j++)
    {
      blocked_intervals[5 * i + j] = blocked_intervals_p[i][j];
    }
  }

  for (size_t i = 0; i < 2; i++)
  {
    for (size_t j = 0; j < 5; j++)
    {
      blocked_intervals[20 + 5 * i + j] = blocked_intervals_h[i][j];
    }
  }

  for (size_t i = 0; i < 4; i++)
  {
    blocked_intervals[30 + i] = blocked_intervals_sing_p[i];
  }

  InvKinElbowInterval::quickSortLower(blocked_intervals, 0, 33);
  InvKinElbowInterval::mergeSortedIntervals(blocked_intervals, 33);

  // Compute feasible intervals from blocked intervals
  int ir = -1;
  if ((blocked_intervals[0].lower_limit > -M_PI))
  {
    ir++;
    feasible_intervals[0].setLimits(-M_PI, M_PI);
  }
  for (auto& blocked_interval : blocked_intervals)
  {
    if (blocked_interval.init)
    {
      if (ir >= 0)
      {
        feasible_intervals[ir].upper_limit = blocked_interval.lower_limit;
      }
      if (blocked_interval.upper_limit < M_PI)
      {
        ir++;
        feasible_intervals[ir].setLimits(blocked_interval.upper_limit, M_PI);
      }
    }
    else
    {
      if ((feasible_intervals[ir].upper_limit == M_PI) && (feasible_intervals[0].lower_limit == -M_PI))
      {
        feasible_intervals[ir].overlap = true;
        feasible_intervals[0].overlap = true;
      }

      break;
    }
  }

  n = ir;

  return result;
}

InvKinMsg InvKin::getClosestPositionIK(InvKinJoints sol[], int& index_sol, InvKinJoints& seed_state,
                                       InvKinXCart& seed_state_x, InvKinXCart& cartXPose, int configs[], int nConfigs,
                                       InvKinMsg (*optimize)(InvKinElbowInterval[], int&, InvKinXCart&, InvKinXCart&))
{
  InvKinMsg result[nConfigs];
  InvKinMsg kinematics_return;

  bool init_0 = false;  // calculate helper matrices only one time, only dependent on elbow configuration.
  bool init_1 = false;
  Matrix3d as_0, bs_0, cs_0;  // helper matrices for elbow angle >= 0
  Matrix3d aw_0, bw_0, cw_0;
  Matrix3d as_1, bs_1, cs_1;  // helper matrices for elbow angle < 0
  Matrix3d aw_1, bw_1, cw_1;

  bool gc_4_seed;                             // elbow configuration parameter (GC) in seed_state
  gc_4_seed = (seed_state_x.config & 2) > 0;  // true for negative angle
  InvKinXCart seed_state_loc = seed_state_x;

  InvKinElbowInterval feasible_intervals[34];
  int n;
  double dist_from_seed[nConfigs];
  index_sol = 0;

  for (size_t i = 0; i < nConfigs; i++)
  {
    if ((configs[i] & 2) == 0)
    {  // elbow joint angle zero/positive if second bit is not set

      seed_state_loc.config = configs[i];
      cartXPose.config = configs[i];

      if (!init_0)
      {
        inverseKinematics(sol[i], cartXPose, as_0, bs_0, cs_0, aw_0, bw_0, cw_0);
        init_0 = true;
      }

      if (gc_4_seed)
      {                                                        // seed elbow angle negative
        seed_state_loc.nsparam = seed_state_x.nsparam + M_PI;  // change nsparam if elbow configurations differ
                                                               // (different reference planes for calculation of
                                                               // nsparam)
        if (seed_state_loc.nsparam > M_PI)
        {
          seed_state_loc.nsparam = -M_PI + fmod(seed_state_loc.nsparam, M_PI);
        }
      }
      else
      {
        seed_state_loc.nsparam = seed_state_x.nsparam;
      }

      computeFeasibleIntervals(feasible_intervals, cartXPose, as_0, bs_0, cs_0, aw_0, bw_0, cw_0, n);

      if (optimize(feasible_intervals, n, seed_state_loc, cartXPose) == InvKin_NO_SOLUTION_FOR_ELBOW)
      {
        result[i] = (InvKinMsg)(InvKin_WARNING | InvKin_NO_SOLUTION_FOR_ELBOW);
        dist_from_seed[i] = 1000.0;
      }
      else
      {
        result[i] = inverseKinematics(sol[i], cartXPose);

        dist_from_seed[i] = 0.0;
        for (size_t j = 0; j < NR_JOINTS; j++)
        {
          dist_from_seed[i] = dist_from_seed[i] + fabs(sol[i][j] - seed_state[j]);
        }
      }
    }
    else
    {
      seed_state_loc.config = configs[i];
      cartXPose.config = configs[i];

      if (!init_1)
      {
        inverseKinematics(sol[i], cartXPose, as_1, bs_1, cs_1, aw_1, bw_1, cw_1);
        init_1 = true;
      }

      if (!gc_4_seed)
      {                                                        // seed elbow angle positive
        seed_state_loc.nsparam = seed_state_x.nsparam + M_PI;  // change nsparam if elbow configurations differ
                                                               // (different reference planes for calculation of
                                                               // nsparam)
        if (seed_state_loc.nsparam > M_PI)
        {
          seed_state_loc.nsparam = -M_PI + fmod(seed_state_loc.nsparam, M_PI);
        }
      }
      else
      {
        seed_state_loc.nsparam = seed_state_x.nsparam;
      }

      computeFeasibleIntervals(feasible_intervals, cartXPose, as_1, bs_1, cs_1, aw_1, bw_1, cw_1, n);

      if (optimize(feasible_intervals, n, seed_state_loc, cartXPose) == InvKin_NO_SOLUTION_FOR_ELBOW)
      {
        result[i] = (InvKinMsg)(InvKin_WARNING | InvKin_NO_SOLUTION_FOR_ELBOW);
        dist_from_seed[i] = 1000.0;
      }
      else
      {
        result[i] = inverseKinematics(sol[i], cartXPose);

        dist_from_seed[i] = 0.0;
        for (size_t j = 0; j < NR_JOINTS; j++)
        {
          dist_from_seed[i] = dist_from_seed[i] + fabs(sol[i][j] - seed_state[j]);
        }
      }
    }

    if (dist_from_seed[index_sol] >= dist_from_seed[i])
    {  // choose solution with lowest distance from seed
      index_sol = i;
    }
  }

  return result[index_sol];
}

InvKinMsg InvKin::redundancyResolutionExp(InvKinElbowInterval feasible_intervals[], int& n, InvKinXCart& seed_state_x,
                                          InvKinXCart& cartXPose)
{
  InvKinMsg result = InvKin_OK;

  // choose "optimal" nullspace parameter

  // determine current/nearest interval
  int current_interval = -1;
  int tmp = -1;
  int i = 0;
  if (feasible_intervals[0].init)
  {
    for (i = 0; i <= n; ++i)
    {
      if (seed_state_x.nsparam <= feasible_intervals[i].upper_limit)
      {
        if (seed_state_x.nsparam >= feasible_intervals[i].lower_limit)
        {
          current_interval = i;
          i++;
          break;
        }
        else
        {
          tmp = i;
          i++;
          break;  // no further suitable interval (non-overlapping intervals)
        }
      }
    }

    i = i - 1;
  }
  else
  {
    i = -1;
  }

  double k = 0.01;  // 0.5
  double a = 10.0;
  double tmp_lower_limit;
  double tmp_upper_limit;
  double tmp_current;

  if (current_interval > -1)
  {
    if (!feasible_intervals[current_interval].overlap)
    {
      cartXPose.nsparam =
          seed_state_x.nsparam +
          k * (feasible_intervals[current_interval].upper_limit - feasible_intervals[current_interval].lower_limit) /
              2.0 * (exp(-a * (seed_state_x.nsparam - feasible_intervals[current_interval].lower_limit) /
                         ((feasible_intervals[current_interval].upper_limit -
                           feasible_intervals[current_interval].lower_limit))) -
                     (exp(-a * (feasible_intervals[current_interval].upper_limit - seed_state_x.nsparam) /
                          (feasible_intervals[current_interval].upper_limit -
                           feasible_intervals[current_interval].lower_limit))));
    }
    else
    {
      if (current_interval == 0)
      {  // nsparam in feasible interval starting from -Pi
        if (n == 0)
        {  // only one feasible interval with overlap --> all nsparams feasible, keep current one.
          cartXPose.nsparam = seed_state_x.nsparam;
        }
        else
        {
          tmp_lower_limit = feasible_intervals[n].lower_limit;
          tmp_upper_limit = M_PI + (feasible_intervals[0].upper_limit + M_PI);
          tmp_current = M_PI + (seed_state_x.nsparam + M_PI);

          cartXPose.nsparam = tmp_current +
                              k * (tmp_upper_limit - tmp_lower_limit) / 2.0 *
                                  (exp(-a * (tmp_current - tmp_lower_limit) / ((tmp_upper_limit - tmp_lower_limit))) -
                                   (exp(-a * (tmp_upper_limit - tmp_current) / (tmp_upper_limit - tmp_lower_limit))));

          // map temporal nsparam to interval -Pi..Pi
          if (cartXPose.nsparam >= M_PI)
            cartXPose.nsparam = -M_PI + fmod(cartXPose.nsparam, M_PI);
        }
      }
      else  // nsparam in feasible interval ending at Pi
      {
        tmp_lower_limit = feasible_intervals[n].lower_limit;
        tmp_upper_limit = M_PI + (feasible_intervals[0].upper_limit + M_PI);
        tmp_current = seed_state_x.nsparam;

        cartXPose.nsparam = tmp_current +
                            k * (tmp_upper_limit - tmp_lower_limit) / 2.0 *
                                (exp(-a * (tmp_current - tmp_lower_limit) / ((tmp_upper_limit - tmp_lower_limit))) -
                                 (exp(-a * (tmp_upper_limit - tmp_current) / (tmp_upper_limit - tmp_lower_limit))));

        // map temporal nsparam to interval -Pi..Pi
        if (cartXPose.nsparam >= M_PI)
          cartXPose.nsparam = -M_PI + fmod(cartXPose.nsparam, M_PI);
      }
    }
  }
  else
  {
    if (i > -1)
    {
      // Nullspace-Parameter not in feasible region, setting it to nearest feasible region
      if (tmp > -1)
      {  // nsparam between two feasible intervals
        if (i > 0)
        {
          if (abs(seed_state_x.nsparam - feasible_intervals[i].lower_limit) <=
              abs(seed_state_x.nsparam - feasible_intervals[i - 1].upper_limit))
            cartXPose.nsparam = feasible_intervals[i].lower_limit;
          else
            cartXPose.nsparam = feasible_intervals[i - 1].upper_limit;
        }
        else  // nsparam in overlap region from M_PI to -M_PI
        {
          if (abs(feasible_intervals[0].lower_limit - seed_state_x.nsparam) <=
              (abs(feasible_intervals[0].lower_limit + M_PI) + (M_PI - feasible_intervals[n].upper_limit)))
            cartXPose.nsparam = feasible_intervals[0].lower_limit;
          else
            cartXPose.nsparam = feasible_intervals[n].upper_limit;
        }
      }
      else  // nsparam over last feasible interval
      {
        if (abs(seed_state_x.nsparam - feasible_intervals[i].upper_limit) <=
            (abs(feasible_intervals[0].lower_limit + M_PI) + (M_PI - feasible_intervals[i].upper_limit)))
          cartXPose.nsparam = feasible_intervals[i].upper_limit;
        else
          cartXPose.nsparam = feasible_intervals[0].lower_limit;
      }
    }
    else
    {  // no solution
      return InvKin_NO_SOLUTION_FOR_ELBOW;
    }
  }

  return result;
}

InvKinMsg InvKin::getIKefuncFixedConfig(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>& solution)
{
  // fixed config is determined in ik_pose

  InvKinXCart seed_state_x;
  forwardKinematics(seed_state_x, seed_state);  // determine nsparam for current pose/seed_state

  // solve inverse kinematics first for nsparam=0.0 to get provisional solution and helper matrices As, ... .
  ik_pose.nsparam = 0.0;

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  Matrix3d as, bs, cs;
  Matrix3d aw, bw, cw;  // helper matrices

  kinematics_return = inverseKinematics(joints, ik_pose, as, bs, cs, aw, bw, cw);

  // determine feasible intervals for nullspace parameter
  InvKinElbowInterval feasible_intervals[34];
  int n;  // array index of last initialized feasible interval
  computeFeasibleIntervals(feasible_intervals, ik_pose, as, bs, cs, aw, bw, cw, n);

  // choose "optimal" nullspace parameter
  redundancyResolutionExp(feasible_intervals, n, seed_state_x, ik_pose);

  // solution
  kinematics_return = inverseKinematics(joints, ik_pose);
  solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}

InvKinMsg InvKin::getIKefuncFixedConfigFixedNs(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>& solution)
{
  // fixed config and nsparam are determined in ik_pose

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  // solution
  kinematics_return = inverseKinematics(joints, ik_pose);
  solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}

InvKinMsg InvKin::getIKefunc(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>& solution)
{
  InvKinXCart seed_state_x;
  forwardKinematics(seed_state_x, seed_state);  // determine nsparam for current pose/seed_state

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  int configs[8];
  InvKinJoints joints_arr[8];
  int index_sol;
  int counter = 0;
  int counter_tmp = 0;

  configs[counter] = seed_state_x.config;  // keep in current config (preferred)
  counter++;

  // only check for different configurations if respective joint angle is close to zero
  if (abs(seed_state[1]) <= 0.1)
  {
    configs[counter] = configs[0] ^ (1 << 0);  // toggle first bit
    counter++;
  }
  if (abs(seed_state[3]) <= 0.1)
  {
    counter_tmp = counter;
    for (int i = 0; i < counter_tmp; i++)
    {
      configs[counter] = configs[i] ^ (1 << 1);  // toggle second bit
      counter++;
    }
  }
  if (abs(seed_state[5]) <= 0.5)
  {
    counter_tmp = counter;
    for (int i = 0; i < counter_tmp; i++)
    {
      configs[counter] = configs[i] ^ (1 << 2);  // toggle third bit
      counter++;
    }
  }

  kinematics_return = getClosestPositionIK(joints_arr, index_sol, seed_state, seed_state_x, ik_pose, configs, counter,
                                          redundancyResolutionExp);

  if (kinematics_return != InvKin_OK && kinematics_return != (InvKin_WARNING | InvKin_CLOSE_TO_SINGULARITY))
  {  // check all configurations if no solution was found

    configs[0] = 0;
    configs[1] = 1;
    configs[2] = 2;
    configs[3] = 3;
    configs[4] = 4;
    configs[5] = 5;
    configs[6] = 6;
    configs[7] = 7;
    kinematics_return = getClosestPositionIK(joints_arr, index_sol, seed_state, seed_state_x, ik_pose, configs, 8,
                                            redundancyResolutionExp);
  }

  joints = joints_arr[index_sol];
  solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}
