/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
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
#ifndef INCLUDE_RLL_MOVE_PERMISSIONS_H_
#define INCLUDE_RLL_MOVE_PERMISSIONS_H_

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <stdint.h>
#include <ros/ros.h>

/**
 * Simple permission management to verify if an operation, e.g. a service call,
 * is permitted in the current context.
 *
 * Permissions are kept lightweight. A 32-bit integer is used to represent a set
 * of bitwise permissions, which limits the amount of available permissions to 32.
 * A permission first needs to registered and is assigned an "permission index",
 * the corresponding bit position in the 32-bit integer. This permission can now
 * be granted or revoked. A group of permissions is described as the combination of
 * multiple permission indices and is internally used as a bitmask.
 *
 * Furthermore, operations can be registered by specifying a name and a set of requirements,
 * in form of permissions, that need to be met in order for this operation to be permitted.
 * To avoid having to register every possible operation, default requirements can
 * be specified which will need to met for every unregistered operation.
 *
 */
class Permissions
{
public:
  using Group = uint32_t;
  using Index = uint8_t;
  static const Index MAX_PERMISSION_INDEX = 32 - 1;
  static const Group NO_PERMISSION_REQUIRED = 0;
  // since the zeroth bit cannot be set this permission will never be granted
  static const Group DENY_ALL = 1;

  explicit Permissions() = default;

  bool isPremitted(Index index) const
  {
    if (index > MAX_PERMISSION_INDEX)
    {
      return false;
    }

    return (current_permissions_ & index == index);
  }

  bool areAllRequiredPermissionsSet(Group permissions) const
  {
    return (current_permissions_ & permissions) == permissions;
  }

  Index registerPermission(const std::string& name, bool isPermitted = false)
  {
    if (permissions_count_ >= MAX_PERMISSION_INDEX)
    {
      ROS_ERROR("Cannot add new permission! Maximum amount of permission registered");
      return 0;
    }

    permission_names_.push_back(name);
    Index index = permissions_count_++;
    updatePermission(index, isPermitted);
    ROS_DEBUG("Registering permission: %s -> index=%d", name.c_str(), index);

    return index;
  }

  bool updatePermission(const std::string& name, bool value)
  {
    const auto iter = std::find(permission_names_.begin(), permission_names_.end(), name);
    if (iter == permission_names_.end())
    {
      ROS_ERROR("No permission named %s", name.c_str());
      return false;
    }
    Index index = std::distance(permission_names_.begin(), iter);
    return updatePermission(index, value);
  }

  void clearAllPermissions()
  {
    current_permissions_ = DENY_ALL;
  }

  bool updatePermission(Index index, bool is_permitted)
  {
    // the zeroth bit can never be set, also only registered permissions can be updated
    if (index < 1 || index > permissions_count_)
    {
      ROS_ERROR("Cannot update permission for index: %d", index);
      return false;
    }

    if (is_permitted)
    {
      current_permissions_ |= index;
    }
    else
    {
      current_permissions_ &= ~(index);
    }

    ROS_DEBUG("Updating permission: %d=%s, current: %u", index, is_permitted ? "yes" : "no", current_permissions_);

    return true;
  }

  void setRequiredPermissionsFor(std::string name, Group requirements)
  {
    requirements_by_name[name] = requirements;
  }

  void setDefaultRequiredPermissions(Group requirements)
  {
    default_requirements_ = requirements;
    ROS_DEBUG("Update default requirements=%u", default_requirements_);
  }

  bool areAllRequirementsMetFor(std::string name) const
  {
    Group requirements = default_requirements_;

    // check if explicit requirements have been set for this name
    const auto iter = requirements_by_name.find(name);
    if (iter != requirements_by_name.end())
    {
      requirements = iter->second;
    }

    bool permitted = areAllRequiredPermissionsSet(requirements);
    ROS_DEBUG("%s requirements=%u, current=%u, permitted? %s", name.c_str(), requirements, current_permissions_,
              permitted ? "yes" : "no");

    return permitted;
  }

private:
  Group current_permissions_ = 0;

  // reserve the zeroth bit; it cannot be set
  std::vector<std::string> permission_names_{ "forbidden" };
  uint8_t permissions_count_ = 1;

  // by default nothing is permitted
  Group default_requirements_ = DENY_ALL;
  std::map<std::string, Group> requirements_by_name;
};

#endif /* INCLUDE_RLL_MOVE_PERMISSIONS_H_ */
