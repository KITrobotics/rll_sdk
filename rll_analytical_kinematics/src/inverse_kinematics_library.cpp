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

#include <rll_analytical_kinematics/inverse_kinematics_library.h>

// InvKin properties
double InvKin::LIMBS[4];
InvKinJoints InvKin::LOWER_JOINT_LIMITS;
InvKinJoints InvKin::UPPER_JOINT_LIMITS;
bool InvKin::INITIALIZED = false;

// Function to generate Cross product matrix:
inline Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& mat)
{
  Eigen::Matrix3d result;
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

const double InvKinJoints::operator()(int i) const
{
  return this->j[i];
}

InvKinFrame::InvKinFrame(const double d, const double theta, const double a, const double alpha)
{
  double ca = cos(alpha);
  double sa = sin(alpha);
  double ct = cos(theta);
  double st = sin(theta);

  ori << ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca;

  pos << a * ct, a * st, d;
}

void InvKinFrame::setQuaternion(const double w, const double x, const double y, const double z)
{
  ori = static_cast<Eigen::Matrix3d>(Eigen::Quaterniond(w, x, y, z));
}

void InvKinFrame::getQuaternion(double* w, double* x, double* y, double* z) const
{
  Eigen::Quaterniond q = static_cast<Eigen::Quaterniond>(ori);
  *w = q.w();
  *x = q.x();
  *y = q.y();
  *z = q.z();
}

void InvKinFrame::setPosition(const double x, const double y, const double z)
{
  pos << x, y, z;
}

InvKinFrame InvKinFrame::operator*(const InvKinFrame t) const
{
  InvKinFrame result;
  result.ori = this->ori * t.ori;
  result.pos = this->ori * t.pos + this->pos;
  return result;
}

bool InvKinElbowInterval::operator<(const InvKinElbowInterval& rhs) const
{
  // non-initialized intervals are bigger
  if (!init)
  {
    return false;
  }
  if (init && !rhs.init)
  {
    return true;
  }
  if (lower_limit < rhs.lower_limit)
  {
    return true;
  }
  return false;
}

void InvKinElbowInterval::mergeSortedIntervals(InvKinElbowInterval intervals[], const int n)
{
  std::vector<InvKinElbowInterval> stack;

  stack.push_back(intervals[0]);

  for (size_t i = 1; i < n; i++)
  {
    if (!intervals[i].init)
    {
      break;
    }

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
                                                         InvKinElbowInterval blocked_intervals[], const double an,
                                                         const double bn, const double cn, const double ad,
                                                         const double bd, const double cd, const int size)
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
      if (std::signbit(interval_limits[j].add) ==
          std::signbit(interval_limits[j].derivativePivot(an, bn, cn, ad, bd, cd)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

        continue;
      }
      else
      {
        if (std::signbit(interval_limits[j + 1].add) !=
            std::signbit(interval_limits[j + 1].derivativePivot(an, bn, cn, ad, bd, cd)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

          continue;
        }
      }
    }
    else
    {
      if (std::signbit(interval_limits[j].add) ==
          std::signbit(interval_limits[j].derivativePivot(an, bn, cn, ad, bd, cd)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
        blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

        break;
      }
      else
      {
        if (std::signbit(interval_limits[0].add) !=
            std::signbit(interval_limits[0].derivativePivot(an, bn, cn, ad, bd, cd)))
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
                                                         InvKinElbowInterval blocked_intervals[], const double a,
                                                         const double b, const double c, const double gc_h,
                                                         const int size)
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
      if (std::signbit(interval_limits[j].add) == std::signbit(interval_limits[j].derivativeHinge(a, b, c, gc_h)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

        continue;
      }
      else
      {
        if (std::signbit(interval_limits[j + 1].add) !=
            std::signbit(interval_limits[j + 1].derivativeHinge(a, b, c, gc_h)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, interval_limits[j + 1].lower_limit);

          continue;
        }
      }
    }
    else
    {
      if (std::signbit(interval_limits[j].add) == std::signbit(interval_limits[j].derivativeHinge(a, b, c, gc_h)))
      {  // compare if signs differ
        blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
        blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

        break;
      }
      else
      {
        if (std::signbit(interval_limits[0].add) != std::signbit(interval_limits[0].derivativeHinge(a, b, c, gc_h)))
        {
          blocked_intervals[j].setLimits(interval_limits[j].lower_limit, M_PI);
          blocked_intervals[j + 1].setLimits(-M_PI, interval_limits[0].lower_limit);

          break;
        }
      }
    }
  }
}

void InvKinElbowInterval::mapLimitsToElbowAnglePivot(InvKinElbowInterval interval_limits[],
                                                     const double lower_joint_limit, const double upper_joint_limit,
                                                     const double an, const double bn, const double cn, const double ad,
                                                     const double bd, const double cd, const double gc_p,
                                                     int* size_init)
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

  *size_init = 0;
  for (size_t j = 0; j < 4; j++)
  {  // check if calculated elbow-limit is matching to limit in joint-space
    if (fabs(interval_limits[j].add - interval_limits[j].jointAnglePivot(an, bn, cn, ad, bd, cd, gc_p)) >= 1E-06)
    {
      interval_limits[j].reset();
    }
    if (interval_limits[j].init)
    {
      (*size_init)++;
    }
  }
}

void InvKinElbowInterval::mapLimitsToElbowAngleHinge(InvKinElbowInterval interval_limits[],
                                                     const double lower_joint_limit, const double upper_joint_limit,
                                                     const double a, const double b, const double c, const double gc_h,
                                                     int* size_init)
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

  *size_init = 0;
  for (size_t j = 0; j < 4; j++)
  {  // check if calculated elbow-limit is matching to limit in joint-space
    if (fabs(interval_limits[j].add - interval_limits[j].jointAngleHinge(a, b, c, gc_h)) >= 1E-06)
    {
      interval_limits[j].reset();
    }
    if (interval_limits[j].init)
    {
      (*size_init)++;
    }
  }
}

inline double InvKinElbowInterval::derivativePivot(const double an, const double bn, const double cn, const double ad,
                                                   const double bd, const double cd)
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

inline double InvKinElbowInterval::jointAnglePivot(const double an, const double bn, const double cn, const double ad,
                                                   const double bd, const double cd, const double gc)
{
  return atan2(gc * (an * sin(lower_limit) + bn * cos(lower_limit) + cn),
               gc * (ad * sin(lower_limit) + bd * cos(lower_limit) + cd));
}

inline double InvKinElbowInterval::derivativeHinge(const double a, const double b, const double /*c*/, const double gc)
{
  return -gc * ((a * cos(lower_limit) - b * sin(lower_limit)) /
                (fabs(sin(add))));  // division by zero impossible, function is only called in joint limit != 0 or Pi.
}

inline double InvKinElbowInterval::jointAngleHinge(const double a, const double b, const double c, const double gc)
{
  return gc * acos(a * sin(lower_limit) + b * cos(lower_limit) + c);
}

InvKinMsg InvKin::forwardKinematics(InvKinXCart* cart_pose, const InvKinJoints& joint_angles)
{
  InvKinMsg result = INVKIN_OK;

  InvKinFrame mbs;  // shoulder pose
  InvKinFrame mbe;  // elbow pose
  InvKinFrame mbw;  // wrist pose
  InvKinFrame mbf;  // flange pose

  mbs = InvKinFrame(LIMBS[0], joint_angles(0), 0.0, -M_PI / 2.0) * InvKinFrame(0.0, joint_angles(1), 0.0, M_PI / 2.0);
  mbe = mbs * InvKinFrame(LIMBS[1], joint_angles(2), 0.0, M_PI / 2.0) *
        InvKinFrame(0.0, joint_angles(3), 0.0, -M_PI / 2.0);
  mbw = mbe * InvKinFrame(LIMBS[2], joint_angles(4), 0.0, -M_PI / 2.0) *
        InvKinFrame(0.0, joint_angles(5), 0.0, M_PI / 2.0);
  mbf = mbw * InvKinFrame(LIMBS[3], joint_angles(6), 0.0, 0.0);

  cart_pose->pose = mbf;

  // determine configuration
  cart_pose->config = (static_cast<int>(joint_angles(1) < 0) << 0) | (static_cast<int>(joint_angles(3) < 0) << 1) |
                      (static_cast<int>(joint_angles(5) < 0) << 2);

  // determine reference plane and null-space-parameter (elbow-redundancy)

  Eigen::Vector3d xsw = mbw.pos - mbs.pos;  // Vector shoulder to wrist
  Eigen::Vector3d xsw_n = xsw;              // normalized
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
  double joint_angle_2_v = acos(((LIMBS[1] * LIMBS[1]) + (lsw * lsw) - (LIMBS[2] * LIMBS[2])) /
                                (2 * LIMBS[1] * lsw));  // virtual shoulder joint angle, only intermediate step

  if ((cart_pose->config & 2) == 0)
  {  // elbow joint angle zero/positive if second bit is not set
    joint_angle_2_v = atan2(sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)), xsw(2)) + joint_angle_2_v;
  }
  else
  {
    joint_angle_2_v = atan2(sqrt(xsw(0) * xsw(0) + xsw(1) * xsw(1)), xsw(2)) - joint_angle_2_v;
  }

  InvKinFrame mbs_v;  // virtual elbow pose
  mbs_v = InvKinFrame(LIMBS[0], joint_angle_1_v, 0.0, -M_PI / 2.0) * InvKinFrame(0.0, joint_angle_2_v, 0.0, M_PI / 2.0);
  Eigen::Vector3d xse_v;  // virtual shoulder to elbow vector
  xse_v << 0.0, 0.0, LIMBS[1];
  Eigen::Vector3d xseb_v;  // virtual shoulder to elbow vector in base coordinates
  xseb_v = mbs_v.ori * xse_v;
  Eigen::Vector3d xseb_n_v = xseb_v;  // normalized
  xseb_n_v.normalize();

  Eigen::Vector3d v_sew_v;  // virtual reference plane (shoulder, elbow, wrist) normal vector
  v_sew_v = xseb_n_v.cross(xsw_n);
  v_sew_v.normalize();

  Eigen::Vector3d v_sew;   // real reference plane (shoulder, elbow, wrist) normal vector
  Eigen::Vector3d xseb_n;  // real normalized shoulder to elbow vector in base coordinates
  xseb_n = mbe.pos - mbs.pos;
  xseb_n.normalize();  // TODO(updim): nomalize necessary in this context??
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
      cart_pose->nsparam = 0.0;
    }
    else if (nsparam_temp <= -1.0)
    {
      cart_pose->nsparam = M_PI;
    }
    else
    {
      cart_pose->nsparam = acos(nsparam_temp);
    }
  }
  else
  {
    if (nsparam_temp >= 1.0)
    {
      cart_pose->nsparam = -0.0;
    }
    else if (nsparam_temp <= -1.0)
    {
      cart_pose->nsparam = -M_PI;
    }
    else
    {
      cart_pose->nsparam = -acos(nsparam_temp);
    }
  }

  return result;
}

InvKinMsg InvKin::inverseKinematics(InvKinJoints* joint_angles, const InvKinXCart& cart_pose)
{
  Eigen::Matrix3d as, bs, cs;  // helper matrix As, Bs, Cs
  Eigen::Matrix3d aw, bw, cw;  // helper matrix Aw, Bw, Cw

  return InvKin::inverseKinematics(joint_angles, cart_pose, &as, &bs, &cs, &aw, &bw, &cw, true);
}

InvKinMsg InvKin::inverseKinematics(InvKinJoints* joint_angles, const InvKinXCart& cart_pose, Eigen::Matrix3d* as,
                                    Eigen::Matrix3d* bs, Eigen::Matrix3d* cs, Eigen::Matrix3d* aw, Eigen::Matrix3d* bw,
                                    Eigen::Matrix3d* cw, const bool check_limits)
{
  InvKinMsg result = INVKIN_OK;

  InvKinFrame mfw;  // wrist pose in flange coordinates
  mfw.pos[2] = -LIMBS[3];

  Eigen::Vector3d xw = (cart_pose.pose * mfw).pos;  // wrist position in base coordinates
  Eigen::Vector3d xs(0, 0, LIMBS[0]);               // shoulder pos

  Eigen::Vector3d xsw = xw - xs;  // vector from shoulder to wrist
  double lsw = xsw.norm();

  // check if target is too close/far
  if (lsw > LIMBS[1] + LIMBS[2])
  {
    result = static_cast<InvKinMsg>(INVKIN_ERROR | INVKIN_TARGET_TOO_FAR);
    return result;
  }

  if (lsw < fabs(LIMBS[1] - LIMBS[2]))
  {
    result = static_cast<InvKinMsg>(INVKIN_ERROR | INVKIN_TARGET_TOO_CLOSE);
    return result;
  }

  Eigen::Vector3d xsw_n = xsw;  // normalized shoulder to wrist vector
  xsw_n.normalize();
  Eigen::Matrix3d xsw_n_cross = crossMatrix(xsw_n);  // cross product matrix of xsw_n

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

  double joint_angle_2_v = acos(((LIMBS[1] * LIMBS[1]) + (lsw * lsw) - (LIMBS[2] * LIMBS[2])) /
                                (2 * LIMBS[1] * lsw));  // virtual shoulder joint angle, only intermediate step

  // determine elbow angle using the law of cosines and virtual shoulder joint angle, depending on configuration
  if ((cart_pose.config & 2) == 0)
  {  // elbow joint angle zero/positive if second bit is not set
    (*joint_angles)[3] =
        acos(((lsw * lsw) - (LIMBS[1] * LIMBS[1]) - (LIMBS[2] * LIMBS[2])) / (2 * LIMBS[1] * LIMBS[2]));
    joint_angle_2_v = atan2(sqrt((xsw(0) * xsw(0)) + (xsw(1) * xsw(1))), xsw(2)) + joint_angle_2_v;
  }
  else
  {
    (*joint_angles)[3] =
        -acos(((lsw * lsw) - (LIMBS[1] * LIMBS[1]) - (LIMBS[2] * LIMBS[2])) / (2 * LIMBS[1] * LIMBS[2]));
    joint_angle_2_v = atan2(sqrt((xsw(0) * xsw(0)) + (xsw(1) * xsw(1))), xsw(2)) - joint_angle_2_v;
  }

  InvKinFrame mbu_v;  // virtual upper arm pose
  mbu_v = InvKinFrame(LIMBS[0], joint_angle_1_v, 0.0, -M_PI / 2.0) *
          InvKinFrame(0.0, joint_angle_2_v, 0.0, M_PI / 2.0) * InvKinFrame(LIMBS[1], 0.0, 0.0, M_PI / 2.0);

  // determine real pose as rotation of virtual pose

  // helper matrix As, Bs, Cs to rotate mbu_v
  *as = xsw_n_cross * mbu_v.ori;
  *bs = -xsw_n_cross * (*as);
  *cs = (xsw_n * xsw_n.transpose()) * mbu_v.ori;

  // real joint angles, depending on configuration:

  if ((cart_pose.config & 1) == 0)
  {  // shoulder joint angle zero/positive if first bit is not set
    (*joint_angles)[0] =
        atan2(((*as)(1, 1) * sin(cart_pose.nsparam) + (*bs)(1, 1) * cos(cart_pose.nsparam) + (*cs)(1, 1)),
              ((*as)(0, 1) * sin(cart_pose.nsparam) + (*bs)(0, 1) * cos(cart_pose.nsparam) + (*cs)(0, 1)));

    (*joint_angles)[1] =
        acos((*as)(2, 1) * sin(cart_pose.nsparam) + (*bs)(2, 1) * cos(cart_pose.nsparam) + (*cs)(2, 1));

    (*joint_angles)[2] =
        atan2(-((*as)(2, 2) * sin(cart_pose.nsparam) + (*bs)(2, 2) * cos(cart_pose.nsparam) + (*cs)(2, 2)),
              -((*as)(2, 0) * sin(cart_pose.nsparam) + (*bs)(2, 0) * cos(cart_pose.nsparam) + (*cs)(2, 0)));
  }
  else
  {
    (*joint_angles)[0] =
        atan2(-((*as)(1, 1) * sin(cart_pose.nsparam) + (*bs)(1, 1) * cos(cart_pose.nsparam) + (*cs)(1, 1)),
              -((*as)(0, 1) * sin(cart_pose.nsparam) + (*bs)(0, 1) * cos(cart_pose.nsparam) + (*cs)(0, 1)));

    (*joint_angles)[1] =
        -acos((*as)(2, 1) * sin(cart_pose.nsparam) + (*bs)(2, 1) * cos(cart_pose.nsparam) + (*cs)(2, 1));

    (*joint_angles)[2] =
        atan2(((*as)(2, 2) * sin(cart_pose.nsparam) + (*bs)(2, 2) * cos(cart_pose.nsparam) + (*cs)(2, 2)),
              ((*as)(2, 0) * sin(cart_pose.nsparam) + (*bs)(2, 0) * cos(cart_pose.nsparam) + (*cs)(2, 0)));
  }

  InvKinFrame mue;  // elbow pose in upper arm coordinates, same as virtual elbow pose
  mue = InvKinFrame(0.0, (*joint_angles)[3], 0.0, -M_PI / 2.0);

  // helper matrix Aw, Bw, Cw

  *aw = mue.ori.transpose() * (as->transpose()) * cart_pose.pose.ori;
  *bw = mue.ori.transpose() * (bs->transpose()) * cart_pose.pose.ori;
  *cw = mue.ori.transpose() * (cs->transpose()) * cart_pose.pose.ori;

  if ((cart_pose.config & 4) == 0)
  {  // wrist joint angle zero/positive if third bit is not set
    (*joint_angles)[4] =
        atan2((*aw)(1, 2) * sin(cart_pose.nsparam) + (*bw)(1, 2) * cos(cart_pose.nsparam) + (*cw)(1, 2),
              (*aw)(0, 2) * sin(cart_pose.nsparam) + (*bw)(0, 2) * cos(cart_pose.nsparam) + (*cw)(0, 2));
    (*joint_angles)[5] =
        acos((*aw)(2, 2) * sin(cart_pose.nsparam) + (*bw)(2, 2) * cos(cart_pose.nsparam) + (*cw)(2, 2));
    (*joint_angles)[6] =
        atan2(((*aw)(2, 1) * sin(cart_pose.nsparam) + (*bw)(2, 1) * cos(cart_pose.nsparam) + (*cw)(2, 1)),
              -((*aw)(2, 0) * sin(cart_pose.nsparam) + (*bw)(2, 0) * cos(cart_pose.nsparam) + (*cw)(2, 0)));
  }
  else
  {
    (*joint_angles)[4] =
        atan2(-((*aw)(1, 2) * sin(cart_pose.nsparam) + (*bw)(1, 2) * cos(cart_pose.nsparam) + (*cw)(1, 2)),
              -((*aw)(0, 2) * sin(cart_pose.nsparam) + (*bw)(0, 2) * cos(cart_pose.nsparam) + (*cw)(0, 2)));
    (*joint_angles)[5] =
        -acos((*aw)(2, 2) * sin(cart_pose.nsparam) + (*bw)(2, 2) * cos(cart_pose.nsparam) + (*cw)(2, 2));
    (*joint_angles)[6] =
        atan2(-((*aw)(2, 1) * sin(cart_pose.nsparam) + (*bw)(2, 1) * cos(cart_pose.nsparam) + (*cw)(2, 1)),
              ((*aw)(2, 0) * sin(cart_pose.nsparam) + (*bw)(2, 0) * cos(cart_pose.nsparam) + (*cw)(2, 0)));
  }

  // check for joint limits and singularities depending on check_limits
  if (check_limits)
  {
    result = INVKIN_OK;

    for (int j = 0; j < 7; j++)
    {
      if (((*joint_angles)[j] < LOWER_JOINT_LIMITS[j]) || ((*joint_angles)[j] > UPPER_JOINT_LIMITS[j]))
      {
        result = static_cast<InvKinMsg>(result | INVKIN_WARNING | INVKIN_JOINTLIMIT);
      }
    }

    for (int j = 1; j <= 5; j += 2)
    {
      if (fabs((*joint_angles)[j]) < 15.0 / 180.0 * M_PI)
      {
        result = static_cast<InvKinMsg>(result | INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY);
      }
    }

    Eigen::Vector3d z;
    z << 0, 0, 1;

    double overhead = acos(xsw.dot(z) / lsw);
    if (overhead < 15.0 / 180.0 * M_PI || overhead > 165.0 / 180.0 * M_PI)
    {
      result = static_cast<InvKinMsg>(result | INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY);
    }
  }

  return result;
}

bool InvKin::initialize(const std::vector<double>& joint_distances, const std::vector<double>& lower_joint_limits,
                        const std::vector<double>& upper_joint_limits)
{
  this->LIMBS[0] = joint_distances[0] + joint_distances[1];
  this->LIMBS[1] = joint_distances[2] + joint_distances[3];
  this->LIMBS[2] = joint_distances[4] + joint_distances[5];
  this->LIMBS[3] = joint_distances[6] + joint_distances[7];

  this->LOWER_JOINT_LIMITS.setJoints(lower_joint_limits);
  this->UPPER_JOINT_LIMITS.setJoints(upper_joint_limits);

  INITIALIZED = true;

  return true;
}

InvKinMsg InvKin::computeFeasibleIntervals(InvKinElbowInterval feasible_intervals[], const InvKinXCart& cart_pose,
                                           const Eigen::Matrix3d& as, const Eigen::Matrix3d& bs,
                                           const Eigen::Matrix3d& cs, const Eigen::Matrix3d& aw,
                                           const Eigen::Matrix3d& bw, const Eigen::Matrix3d& cw, int* n)
{
  InvKinMsg result = INVKIN_OK;
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
  if ((cart_pose.config & 1) == 0)
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

  if ((cart_pose.config & 4) == 0)
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
  an[0] = as(1, 1);
  bn[0] = bs(1, 1);
  cn[0] = cs(1, 1);
  ad[0] = as(0, 1);
  bd[0] = bs(0, 1);
  cd[0] = cs(0, 1);

  an[1] = -as(2, 2);
  bn[1] = -bs(2, 2);
  cn[1] = -cs(2, 2);
  ad[1] = -as(2, 0);
  bd[1] = -bs(2, 0);
  cd[1] = -cs(2, 0);

  an[2] = aw(1, 2);
  bn[2] = bw(1, 2);
  cn[2] = cw(1, 2);
  ad[2] = aw(0, 2);
  bd[2] = bw(0, 2);
  cd[2] = cw(0, 2);

  an[3] = aw(2, 1);
  bn[3] = bw(2, 1);
  cn[3] = cw(2, 1);
  ad[3] = -aw(2, 0);
  bd[3] = -bw(2, 0);
  cd[3] = -cw(2, 0);

  a[0] = as(2, 1);
  b[0] = bs(2, 1);
  c[0] = cs(2, 1);
  a[1] = aw(2, 2);
  b[1] = bw(2, 2);
  c[1] = cw(2, 2);

  ///////////////////
  // Pivot Joints //
  //////////////////
  for (size_t i = 0; i < 4; i++)
  {  // for all pivot-joints

    // determine singularity in joint angle
    at = (cn[i] * bd[i] - bn[i] * cd[i]);  // no gc-parameter in at, bt, ct (failure in paper)
    bt = (an[i] * cd[i] - cn[i] * ad[i]);
    ct = (an[i] * bd[i] - bn[i] * ad[i]);

    if (fabs(at * at + bt * bt - ct * ct) <= 1E-06)
    {
      psi_singular = 2 * atan(at / (bt - ct));
      blocked_intervals_sing_p[i].setLimits(
          psi_singular - margin, psi_singular + margin);  // blocked interval due to singularity at psi_singular
    }

    // joint angle limits mapped to elbow angle
    // lower_limit (elbow angle) of InvKinElbowInterval is initialized with corresponding joint angle limit (upper and
    // lower)
    // add of InvKinElbowInterval is used here to remember the corresponding joint angle limit
    InvKinElbowInterval::mapLimitsToElbowAnglePivot(blocked_intervals_limit_p[i], LOWER_JOINT_LIMITS[2 * i],
                                                    UPPER_JOINT_LIMITS[2 * i], an[i], bn[i], cn[i], ad[i], bd[i], cd[i],
                                                    gc_p[i], &counter);

    // sort the lower_limits of InvKinElbowInterval[] --> uninitialized InvKinElbowIntervals move to end
    std::sort(blocked_intervals_limit_p[i], blocked_intervals_limit_p[i] + 4);

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
           UPPER_JOINT_LIMITS[2 * i]) ||
          (blocked_intervals_limit_p[i][0].jointAnglePivot(an[i], bn[i], cn[i], ad[i], bd[i], cd[i], gc_p[i]) <=
           LOWER_JOINT_LIMITS[2 * i]))
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
    InvKinElbowInterval::mapLimitsToElbowAngleHinge(blocked_intervals_limit_h[i], LOWER_JOINT_LIMITS[4 * i + 1],
                                                    UPPER_JOINT_LIMITS[4 * i + 1], a[i], b[i], c[i], gc_h[i], &counter);

    std::sort(blocked_intervals_limit_h[i], blocked_intervals_limit_h[i] + 4);

    if (counter > 0)
    {
      InvKinElbowInterval::determineBlockedIntervalsHinge(blocked_intervals_limit_h[i], blocked_intervals_h[i], a[i],
                                                          b[i], c[i], gc_h[i], 4);
    }
    else
    {
      if ((blocked_intervals_limit_h[i][0].jointAngleHinge(a[i], b[i], c[i], gc_h[i]) >=
           UPPER_JOINT_LIMITS[4 * i + 1]) ||
          (blocked_intervals_limit_h[i][0].jointAngleHinge(a[i], b[i], c[i], gc_h[i]) <= LOWER_JOINT_LIMITS[4 * i + 1]))
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

  std::sort(blocked_intervals, blocked_intervals + 34);
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

  *n = ir;

  return result;
}

InvKinMsg InvKin::getClosestPositionIK(InvKinJoints sol[], int* index_sol, const InvKinJoints& seed_state,
                                       const InvKinXCart& seed_state_x, InvKinXCart* cart_pose, const int configs[],
                                       const int n_configs, InvKinMsg (*optimize)(InvKinElbowInterval[], int,
                                                                                  const InvKinXCart&, InvKinXCart*))
{
  InvKinMsg result[n_configs];
  InvKinMsg kinematics_return;

  bool init_0 = false;  // calculate helper matrices only one time, only dependent on elbow configuration.
  bool init_1 = false;
  Eigen::Matrix3d as_0, bs_0, cs_0;  // helper matrices for elbow angle >= 0
  Eigen::Matrix3d aw_0, bw_0, cw_0;
  Eigen::Matrix3d as_1, bs_1, cs_1;  // helper matrices for elbow angle < 0
  Eigen::Matrix3d aw_1, bw_1, cw_1;

  bool gc_4_seed;                             // elbow configuration parameter (GC) in seed_state
  gc_4_seed = (seed_state_x.config & 2) > 0;  // true for negative angle
  InvKinXCart seed_state_loc = seed_state_x;

  InvKinElbowInterval feasible_intervals[34];
  int n;
  double dist_from_seed[n_configs];
  *index_sol = 0;

  for (size_t i = 0; i < n_configs; i++)
  {
    if ((configs[i] & 2) == 0)
    {  // elbow joint angle zero/positive if second bit is not set

      seed_state_loc.config = configs[i];
      cart_pose->config = configs[i];

      if (!init_0)
      {
        inverseKinematics(&sol[i], *cart_pose, &as_0, &bs_0, &cs_0, &aw_0, &bw_0, &cw_0);
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

      computeFeasibleIntervals(feasible_intervals, *cart_pose, as_0, bs_0, cs_0, aw_0, bw_0, cw_0, &n);

      if (optimize(feasible_intervals, n, seed_state_loc, cart_pose) == INVKIN_NO_SOLUTION_FOR_ELBOW)
      {
        result[i] = static_cast<InvKinMsg>(INVKIN_WARNING | INVKIN_NO_SOLUTION_FOR_ELBOW);
        dist_from_seed[i] = 1000.0;
      }
      else
      {
        result[i] = inverseKinematics(&sol[i], *cart_pose);

        dist_from_seed[i] = 0.0;
        for (size_t j = 0; j < NR_JOINTS; j++)
        {
          dist_from_seed[i] = dist_from_seed[i] + fabs(sol[i][j] - seed_state(j));
        }
      }
    }
    else
    {
      seed_state_loc.config = configs[i];
      cart_pose->config = configs[i];

      if (!init_1)
      {
        inverseKinematics(&sol[i], *cart_pose, &as_1, &bs_1, &cs_1, &aw_1, &bw_1, &cw_1);
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

      computeFeasibleIntervals(feasible_intervals, *cart_pose, as_1, bs_1, cs_1, aw_1, bw_1, cw_1, &n);

      if (optimize(feasible_intervals, n, seed_state_loc, cart_pose) == INVKIN_NO_SOLUTION_FOR_ELBOW)
      {
        result[i] = static_cast<InvKinMsg>(INVKIN_WARNING | INVKIN_NO_SOLUTION_FOR_ELBOW);
        dist_from_seed[i] = 1000.0;
      }
      else
      {
        result[i] = inverseKinematics(&sol[i], *cart_pose);

        dist_from_seed[i] = 0.0;
        for (size_t j = 0; j < NR_JOINTS; j++)
        {
          dist_from_seed[i] = dist_from_seed[i] + fabs(sol[i][j] - seed_state(j));
        }
      }
    }

    if (dist_from_seed[*index_sol] >= dist_from_seed[i])
    {  // choose solution with lowest distance from seed
      *index_sol = i;
    }
  }

  return result[*index_sol];
}

InvKinMsg InvKin::redundancyResolutionExp(InvKinElbowInterval feasible_intervals[], const int n,
                                          const InvKinXCart& seed_state, InvKinXCart* cart_pose)
{
  InvKinMsg result = INVKIN_OK;

  // choose "optimal" nullspace parameter

  // determine current/nearest interval
  int current_interval = -1;
  int tmp = -1;
  int i = 0;
  if (feasible_intervals[0].init)
  {
    for (i = 0; i <= n; ++i)
    {
      if (seed_state.nsparam <= feasible_intervals[i].upper_limit)
      {
        if (seed_state.nsparam >= feasible_intervals[i].lower_limit)
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
      cart_pose->nsparam =
          seed_state.nsparam +
          k * (feasible_intervals[current_interval].upper_limit - feasible_intervals[current_interval].lower_limit) /
              2.0 * (exp(-a * (seed_state.nsparam - feasible_intervals[current_interval].lower_limit) /
                         ((feasible_intervals[current_interval].upper_limit -
                           feasible_intervals[current_interval].lower_limit))) -
                     (exp(-a * (feasible_intervals[current_interval].upper_limit - seed_state.nsparam) /
                          (feasible_intervals[current_interval].upper_limit -
                           feasible_intervals[current_interval].lower_limit))));
    }
    else
    {
      if (current_interval == 0)
      {  // nsparam in feasible interval starting from -Pi
        if (n == 0)
        {  // only one feasible interval with overlap --> all nsparams feasible, keep current one.
          cart_pose->nsparam = seed_state.nsparam;
        }
        else
        {
          tmp_lower_limit = feasible_intervals[n].lower_limit;
          tmp_upper_limit = M_PI + (feasible_intervals[0].upper_limit + M_PI);
          tmp_current = M_PI + (seed_state.nsparam + M_PI);

          cart_pose->nsparam = tmp_current +
                               k * (tmp_upper_limit - tmp_lower_limit) / 2.0 *
                                   (exp(-a * (tmp_current - tmp_lower_limit) / ((tmp_upper_limit - tmp_lower_limit))) -
                                    (exp(-a * (tmp_upper_limit - tmp_current) / (tmp_upper_limit - tmp_lower_limit))));

          // map temporal nsparam to interval -Pi..Pi
          if (cart_pose->nsparam >= M_PI)
          {
            cart_pose->nsparam = -M_PI + fmod(cart_pose->nsparam, M_PI);
          }
        }
      }
      else  // nsparam in feasible interval ending at Pi
      {
        tmp_lower_limit = feasible_intervals[n].lower_limit;
        tmp_upper_limit = M_PI + (feasible_intervals[0].upper_limit + M_PI);
        tmp_current = seed_state.nsparam;

        cart_pose->nsparam = tmp_current +
                             k * (tmp_upper_limit - tmp_lower_limit) / 2.0 *
                                 (exp(-a * (tmp_current - tmp_lower_limit) / ((tmp_upper_limit - tmp_lower_limit))) -
                                  (exp(-a * (tmp_upper_limit - tmp_current) / (tmp_upper_limit - tmp_lower_limit))));

        // map temporal nsparam to interval -Pi..Pi
        if (cart_pose->nsparam >= M_PI)
        {
          cart_pose->nsparam = -M_PI + fmod(cart_pose->nsparam, M_PI);
        }
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
          if (fabs(seed_state.nsparam - feasible_intervals[i].lower_limit) <=
              fabs(seed_state.nsparam - feasible_intervals[i - 1].upper_limit))
          {
            cart_pose->nsparam = feasible_intervals[i].lower_limit;
          }
          else
          {
            cart_pose->nsparam = feasible_intervals[i - 1].upper_limit;
          }
        }
        else  // nsparam in overlap region from M_PI to -M_PI
        {
          if (fabs(feasible_intervals[0].lower_limit - seed_state.nsparam) <=
              (fabs(feasible_intervals[0].lower_limit + M_PI) + (M_PI - feasible_intervals[n].upper_limit)))
          {
            cart_pose->nsparam = feasible_intervals[0].lower_limit;
          }
          else
          {
            cart_pose->nsparam = feasible_intervals[n].upper_limit;
          }
        }
      }
      else  // nsparam over last feasible interval
      {
        if (fabs(seed_state.nsparam - feasible_intervals[i].upper_limit) <=
            (fabs(feasible_intervals[0].lower_limit + M_PI) + (M_PI - feasible_intervals[i].upper_limit)))
        {
          cart_pose->nsparam = feasible_intervals[i].upper_limit;
        }
        else
        {
          cart_pose->nsparam = feasible_intervals[0].lower_limit;
        }
      }
    }
    else
    {  // no solution
      return INVKIN_NO_SOLUTION_FOR_ELBOW;
    }
  }

  return result;
}

InvKinMsg InvKin::getIKefuncFixedConfig(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution)
{
  // fixed config is determined in ik_pose

  InvKinXCart seed_state_x;
  forwardKinematics(&seed_state_x, seed_state);  // determine nsparam for current pose/seed_state

  // solve inverse kinematics first for nsparam=0.0 to get provisional solution and helper matrices As, ... .
  ik_pose.nsparam = 0.0;

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  Eigen::Matrix3d as, bs, cs;
  Eigen::Matrix3d aw, bw, cw;  // helper matrices

  inverseKinematics(&joints, ik_pose, &as, &bs, &cs, &aw, &bw, &cw);

  // determine feasible intervals for nullspace parameter
  InvKinElbowInterval feasible_intervals[34];
  int n;  // array index of last initialized feasible interval
  computeFeasibleIntervals(feasible_intervals, ik_pose, as, bs, cs, aw, bw, cw, &n);

  // choose "optimal" nullspace parameter
  redundancyResolutionExp(feasible_intervals, n, seed_state_x, &ik_pose);

  // solution
  kinematics_return = inverseKinematics(&joints, ik_pose);
  *solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}

InvKinMsg InvKin::getIKfixedNs(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution)
{
  InvKinXCart seed_state_x;
  forwardKinematics(&seed_state_x, seed_state);  // determine nsparam for current pose/seed_state

  InvKinJoints joints;
  InvKinMsg kinematics_return[8];

  double elbow_angle = ik_pose.nsparam;
  if (elbow_angle < -M_PI)
  {  // map elbow angle in interval -Pi..Pi
    elbow_angle = M_PI - fmod(-elbow_angle - M_PI, 2 * M_PI);
  }
  else if (elbow_angle > M_PI)
  {
    elbow_angle = -M_PI + fmod(elbow_angle - M_PI, 2 * M_PI);
  }

  int configs[8];
  InvKinJoints joints_arr[8];
  int index_sol;
  double dist_from_seed[8];
  int counter = 0;

  configs[counter] = seed_state_x.config;  // keep in current config (preferred)
  counter++;
  determineClosestConfigs(configs, &counter, seed_state);

  for (int i = 0; i < 2; i++)
  {
    index_sol = 0;
    dist_from_seed[0] = 0.0;

    for (int j = 0; j < counter; j++)
    {
      ik_pose.config = configs[j];
      ik_pose.nsparam = elbow_angle;
      if ((configs[j] & 2) == 0)
      {                                            // elbow joint angle zero/positive if second bit is not set
        ik_pose.nsparam = ik_pose.nsparam + M_PI;  // change nsparam if elbow joint angle
        if (ik_pose.nsparam > M_PI)                // is positive --> otherwise elbow would
        {                                          // point to bottom for nsparam = 0 due
                                                   // to definition of nsparam
          ik_pose.nsparam = -M_PI + fmod(ik_pose.nsparam, M_PI);
        }
      }

      kinematics_return[j] = inverseKinematics(&joints_arr[j], ik_pose);
      dist_from_seed[j] = 0.0;

      for (size_t k = 0; k < NR_JOINTS; k++)
      {
        dist_from_seed[j] = dist_from_seed[j] + fabs(joints_arr[j][k] - seed_state[k]);
      }
      if (dist_from_seed[index_sol] >= dist_from_seed[j])
      {  // choose solution with lowest distance from seed
        index_sol = j;
      }
    }

    if (kinematics_return[index_sol] != INVKIN_OK &&
        kinematics_return[index_sol] != (INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY))
    {  // check all configurations if no solution was found
      configs[0] = 0;
      configs[1] = 1;
      configs[2] = 2;
      configs[3] = 3;
      configs[4] = 4;
      configs[5] = 5;
      configs[6] = 6;
      configs[7] = 7;
      counter = 8;
      continue;
    }
    break;  // no further iteration if solution was found
  }

  joints = joints_arr[index_sol];
  *solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return[index_sol];
}

InvKinMsg InvKin::getIKefuncFixedConfigFixedNs(InvKinXCart ik_pose, InvKinJoints /*seed_state*/,
                                               std::vector<double>* solution)
{
  // fixed config and nsparam are determined in ik_pose

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  // solution
  kinematics_return = inverseKinematics(&joints, ik_pose);
  *solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}

InvKinMsg InvKin::getIKefunc(InvKinXCart ik_pose, InvKinJoints seed_state, std::vector<double>* solution)
{
  InvKinXCart seed_state_x;
  forwardKinematics(&seed_state_x, seed_state);  // determine nsparam for current pose/seed_state

  InvKinJoints joints;
  InvKinMsg kinematics_return;

  int configs[8];
  InvKinJoints joints_arr[8];
  int index_sol;
  int counter = 0;

  configs[counter] = seed_state_x.config;  // keep in current config (preferred)
  counter++;
  determineClosestConfigs(configs, &counter, seed_state);

  kinematics_return = getClosestPositionIK(joints_arr, &index_sol, seed_state, seed_state_x, &ik_pose, configs, counter,
                                           redundancyResolutionExp);

  if (kinematics_return != INVKIN_OK && kinematics_return != (INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY))
  {  // check all configurations if no solution was found

    configs[0] = 0;
    configs[1] = 1;
    configs[2] = 2;
    configs[3] = 3;
    configs[4] = 4;
    configs[5] = 5;
    configs[6] = 6;
    configs[7] = 7;
    kinematics_return = getClosestPositionIK(joints_arr, &index_sol, seed_state, seed_state_x, &ik_pose, configs, 8,
                                             redundancyResolutionExp);
  }

  joints = joints_arr[index_sol];
  *solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);

  return kinematics_return;
}

void InvKin::determineClosestConfigs(int configs[], int* counter, const InvKinJoints& joint_angles)
{
  if ((*counter) == 1)
  {  // method needs the seed config
    int counter_tmp = 0;
    // only check for different configurations if respective joint angle is close to zero
    if (fabs(joint_angles(1)) <= 0.1)
    {
      configs[(*counter)] = configs[0] ^ (1 << 0);  // toggle first bit
      (*counter)++;
    }
    if (fabs(joint_angles(3)) <= 0.1)
    {
      counter_tmp = (*counter);
      for (int i = 0; i < counter_tmp; i++)
      {
        configs[(*counter)] = configs[i] ^ (1 << 1);  // toggle second bit
        (*counter)++;
      }
    }
    if (fabs(joint_angles(5)) <= 0.5)
    {
      counter_tmp = (*counter);
      for (int i = 0; i < counter_tmp; i++)
      {
        configs[(*counter)] = configs[i] ^ (1 << 2);  // toggle third bit
        (*counter)++;
      }
    }
  }
}

std::ostream& operator<<(std::ostream& stream, InvKinMsg const& val)
{
  stream << "InvKinError: ";

  if ((val & INVKIN_WARNING) != 0)
  {
    stream << "WARN ";
  }

  if ((val & INVKIN_ERROR) != 0)
  {
    stream << "ERROR ";
  }

  if ((val & INVKIN_JOINTLIMIT) != 0)
  {
    stream << "jointlimit ";
  }

  if ((val & INVKIN_TARGET_TOO_FAR) != 0)
  {
    stream << "target_too_far ";
  }

  if ((val & INVKIN_TARGET_TOO_CLOSE) != 0)
  {
    stream << "target_too_close ";
  }

  if ((val & INVKIN_CLOSE_TO_SINGULARITY) != 0)
  {
    stream << "close_to_singularity ";
  }

  if ((val & INVKIN_SINGULARITY) != 0)
  {
    stream << "singularity ";
  }

  if ((val & INVKIN_NO_SOLUTION_FOR_ELBOW) != 0)
  {
    stream << "no_solution_for_elbow ";
  }

  return stream;
}
