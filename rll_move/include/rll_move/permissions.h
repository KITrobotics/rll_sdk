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
#ifndef RLL_MOVE_PERMISSIONS_H
#define RLL_MOVE_PERMISSIONS_H

#include <algorithm>
#include <cstdint>
#include <map>
#include <ros/ros.h>
#include <stack>
#include <string>
#include <vector>

/**
 * Simple permission management to verify if an operation, e.g. a service call,
 * is permitted in the current context.
 *
 * Permissions are kept lightweight. A 32-bit integer is used to represent a set
 * of bitwise permissions, which limits the amount of available permissions to 32.
 * A permission first needs to registered and is assigned an "permission index",
 * which is 2^(bit position in the 32-bit integer). This permission can now
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
  using Index = uint32_t;
  static const uint8_t MAX_PERMISSION_INDICES = 32 - 1;
  static const Group NO_PERMISSION_REQUIRED = 0;
  // since the zeroth bit cannot be set this permission will never be granted
  static const Group DENY_ALL = 1;

  explicit Permissions() = default;

  Group getCurrentPermissions()
  {
    return current_permissions_;
  }

  bool isPermitted(Index index) const
  {
    return (current_permissions_ & index) == index;
  }

  bool areAllRequiredPermissionsSet(Group permissions) const
  {
    return (current_permissions_ & permissions) == permissions;
  }

  Index registerPermission(const std::string& name, bool is_permitted = false)
  {
    if (permissions_count_ >= MAX_PERMISSION_INDICES)
    {
      ROS_ERROR("Cannot add new permission! Maximum amount of permission registered");
      return 0;
    }

    permission_names_.push_back(name);
    Index index = (1U << permissions_count_);
    permissions_count_++;
    updateCurrentPermissions(index, is_permitted);
    ROS_DEBUG("Registering permission: %s -> index=%d", name.c_str(), index);

    return index;
  }

  Index getIndexForName(const std::string& name)
  {
    const auto ITER = std::find(permission_names_.begin(), permission_names_.end(), name);
    if (ITER == permission_names_.end())
    {
      ROS_ERROR("No such permission '%s'", name.c_str());
      return 0U;  // false
    }
    uint8_t bit_position = std::distance(permission_names_.begin(), ITER);
    return (1 << bit_position);
  }

  bool updateCurrentPermissions(const std::string& name, bool value)
  {
    Index index = getIndexForName(name);
    return updateCurrentPermissions(index, value);
  }

  void clearAllPermissions()
  {
    current_permissions_ = DENY_ALL;
  }

  void storeCurrentPermissions()
  {
    stored_permissions_.push(current_permissions_);
  }

  void restorePreviousPermissions()
  {
    if (stored_permissions_.empty())
    {
      ROS_WARN("Cannot restore previous permissions, no permissions have been stored.");
      return;
    }

    current_permissions_ = stored_permissions_.top();
    stored_permissions_.pop();
  }

  bool updateCurrentPermissions(Index index, bool is_permitted)
  {
    // the zeroth bit can never be set, also only registered permissions can be updated
    if (index < 1 || index >= (1U << permissions_count_))
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
      current_permissions_ &= ~index;
    }

    ROS_DEBUG("Updating permission: %d=%s, current: %u", index, is_permitted ? "yes" : "no", current_permissions_);

    return true;
  }

  void setDefaultRequiredPermissions(Group requirements)
  {
    default_requirements_ = requirements;
    ROS_DEBUG("Update default requirements=%u", default_requirements_);
  }

  void setRequiredPermissionsFor(const std::string& name, Group requirements, bool inherit_default_permissions = false)
  {
    if (inherit_default_permissions)
    {
      requirements |= default_requirements_;
    }
    requirements_by_name_[name] = requirements;
  }

  bool isPermissionRequiredFor(const std::string& name, Index requirement)
  {
    return (getRequiredPermissionsFor(name) & requirement) == requirement;
  }

  Group getRequiredPermissionsFor(const std::string& name) const
  {
    Group requirements = default_requirements_;
    // check if explicit requirements have been set for this name
    const auto ITER = requirements_by_name_.find(name);
    if (ITER != requirements_by_name_.end())
    {
      requirements = ITER->second;
    }
    return requirements;
  }

  bool areAllRequiredPermissionsSetFor(const std::string& name) const
  {
    Group requirements = getRequiredPermissionsFor(name);
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
  std::map<std::string, Group> requirements_by_name_;
  std::stack<Group> stored_permissions_;
};

#endif  // RLL_MOVE_PERMISSIONS_H
