/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2019-2020 Mark Weinreuter <mark.weinreuter@kit.edu>
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

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

#include <vector>
#include <tf/tf.h>

class ParamParser
{
public:
  // Static Helper functions to get common objects from the parameter server -> could be moved to util file
  static void pose2dToPose3d(const geometry_msgs::Pose2D& pose2d, geometry_msgs::Pose* pose3d, float z = 0);

  static bool getValuesFromParam(const std::string& name, size_t expected, std::vector<double>* values_ptr);

  static geometry_msgs::Point getPointFromParam(const std::string& name);

  static bool getTripleFromParam(const std::string& name, double* v1, double* v2, double* v3);

  static bool getPose2DFromParam(const std::string& name, geometry_msgs::Pose2D* pose);

  static bool getPose2DasPoseFromParam(const std::string& name, geometry_msgs::Pose* pose, float z = 0);

  static bool getVector3FromParam(const std::string& name, geometry_msgs::Vector3* vec);

  static bool getPointFromParam(const std::string& name, geometry_msgs::Point* point);
};

// Helper functions for project interfaces
bool ParamParser::getValuesFromParam(const std::string& name, size_t expected, std::vector<double>* values_ptr)
{
  ros::param::get(name, *values_ptr);
  return values_ptr->size() == expected;
}

bool ParamParser::getTripleFromParam(const std::string& name, double* v1, double* v2, double* v3)
{
  std::vector<double> values;
  ros::param::get(name, values);
  if (values.size() != 3)
  {
    ROS_WARN("Failed to get '%s' from parameter server", name.c_str());
    return false;
  }
  *v1 = values[0];
  *v2 = values[1];
  *v3 = values[2];

  return true;
}

bool ParamParser::getPose2DFromParam(const std::string& name, geometry_msgs::Pose2D* pose)
{
  return getTripleFromParam(name, &pose->x, &pose->y, &pose->theta);
}

void ParamParser::pose2dToPose3d(const geometry_msgs::Pose2D& pose2d, geometry_msgs::Pose* pose3d, float z)
{
  pose3d->position.x = pose2d.x;
  pose3d->position.y = pose2d.y;
  pose3d->position.z = z;

  tf::Quaternion q_orig{ 0, 0, 0, 1 };  // unit quaternion
  tf::Quaternion q_rot = tf::createQuaternionFromRPY(0, 0, pose2d.theta);
  tf::Quaternion q_new = (q_rot * q_orig).normalize();
  quaternionTFToMsg(q_new, pose3d->orientation);
}

bool ParamParser::getPose2DasPoseFromParam(const std::string& name, geometry_msgs::Pose* pose, float z)
{
  geometry_msgs::Pose2D pose_2d;
  if (!getPose2DFromParam(name, &pose_2d))
  {
    ROS_WARN("Failed to get '%s' from param server", name.c_str());
    return false;
  }
  pose2dToPose3d(pose_2d, pose, z);
  return true;
}

bool ParamParser::getVector3FromParam(const std::string& name, geometry_msgs::Vector3* vec)
{
  return getTripleFromParam(name, &vec->x, &vec->y, &vec->z);
}

bool ParamParser::getPointFromParam(const std::string& name, geometry_msgs::Point* point)
{
  return getTripleFromParam(name, &point->x, &point->y, &point->z);
}

geometry_msgs::Point ParamParser::getPointFromParam(const std::string& name)
{
  std::map<std::string, double> values;
  ros::param::get(name, values);

  auto point = geometry_msgs::Point();
  // TODO(mark): validate if set
  point.x = values["x"];
  point.y = values["y"];
  point.z = values["z"];

  return point;
}
