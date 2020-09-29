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
  static const Index DENY_ALL_BIT = (1U << 0);
  // the second bit indicates if the default permissions are required
  // if it is set, the total required permissions for an action are determined
  // by the explicitly set permissions and the current set of default permissions
  static const Index APPLY_DEFAULTS_BIT = (1U << 1);

  explicit Permissions() = default;

  Group getCurrentPermissions()
  {
    return current_permissions_;
  }

  bool isBitAtPositionSet(Group value, Index pos) const
  {
    uint32_t val = (1U << pos);
    return (value & val) == val;
  }

  bool areBitsSet(Group value, Group bit_mask) const
  {
    return (value & bit_mask) == bit_mask;
  }

  bool isPermitted(Index index) const
  {
    return (current_permissions_ & index) == index;
  }

  bool arePermissionsCurrentlySet(Group permissions) const
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

    // find previous permission with this name
    auto iter = std::find(permission_names_.begin(), permission_names_.end(), name);
    if (iter != permission_names_.end())
    {
      ROS_WARN("Permission with name %s already registered.", name.c_str());
      return std::distance(permission_names_.begin(), iter);
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

  bool updateCurrentPermissions(Index index, bool is_permitted)
  {
    // the zeroth and first bits can never be set, also only registered permissions (less than the current amount of
    // indices) can be updated
    if (areBitsSet(DENY_ALL_BIT | APPLY_DEFAULTS_BIT, index) || index >= (1U << permissions_count_))
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

  void clearAllPermissions()
  {
    current_permissions_ = DENY_ALL_BIT;
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

  void setDefaultRequiredPermissions(Group requirements)
  {
    default_requirements_ = requirements;
    ROS_DEBUG("Update default requirements=%u", default_requirements_);
  }

  void setRequiredPermissionsFor(const std::string& name, Group requirements, bool require_default_permissions = false)
  {
    if (require_default_permissions)
    {
      requirements |= APPLY_DEFAULTS_BIT;  // the indicator bit will be resolved to the current defaults in
                                           // getRequiredPermissionsFor
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

      // if REQUIRE_DEFAULTS_INDICATOR bit is set add the current default permissions
      if (areBitsSet(requirements, APPLY_DEFAULTS_BIT))
      {
        // copy the default requirement bits and, clear the default requirements indicator
        requirements |= default_requirements_;
        requirements &= ~APPLY_DEFAULTS_BIT;
      }
    }
    else
    {
      ROS_DEBUG("No requirements registered for %s, using defaults!", name.c_str());
    }
    return requirements;
  }

  bool areAllRequiredPermissionsSetFor(const std::string& name) const
  {
    Group requirements = getRequiredPermissionsFor(name);
    bool permitted = arePermissionsCurrentlySet(requirements);
    ROS_DEBUG("%s requirements=%u, current=%u, permitted? %s", name.c_str(), requirements, current_permissions_,
              permitted ? "yes" : "no");

    return permitted;
  }

  void debugPermission(const std::string& name)
  {
    Group requirements = getRequiredPermissionsFor(name);
    bool permitted = arePermissionsCurrentlySet(requirements);

    ROS_INFO_STREAM(name << " requires permissions: " << requirements << ", currently permitted=" << permitted);
    ROS_INFO_STREAM(std::setw(2) << "#" << std::setw(10) << "Bit Value" << std::setw(24) << "Permission name"
                                 << std::setw(12) << "Current" << std::setw(12) << name);
    uint32_t bit_pos = 0;
    for (std::string& value : permission_names_)
    {
      bool is_current_bit_set = isBitAtPositionSet(current_permissions_, bit_pos);
      bool is_bit_required = isBitAtPositionSet(requirements, bit_pos);
      ROS_INFO_STREAM(std::setw(2) << bit_pos << std::setw(10) << (1U << bit_pos) << std::setw(24) << value
                                   << std::setw(12) << is_current_bit_set << std::setw(12) << is_bit_required);
      bit_pos++;
    }
  }

private:
  Group current_permissions_ = 0;

  // reserve the zeroth and first bit -> they have special meaning and cannot be set
  std::vector<std::string> permission_names_{ "forbidden", "require_defaults" };
  uint8_t permissions_count_ = 2;

  // by default nothing is permitted
  Group default_requirements_ = DENY_ALL_BIT;
  std::map<std::string, Group> requirements_by_name_;
  // push/pop the current set permissions
  std::stack<Group> stored_permissions_;
};

#endif  // RLL_MOVE_PERMISSIONS_H
