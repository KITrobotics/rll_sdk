/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2020 Mark Weinreuter <mark.weinreuter@kit.edu>
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

#include <memory>
#include <rll_move/grasp_object.h>
#include <rll_move/grasp_util.h>
#include <rll_move/move_iface_default.h>
#include <rll_move/move_iface_gripper.h>
#include <rll_move/move_iface_simulation.h>

#define MODEL_HEIGHT (0.012 + 0.025)
#define MODEL_SCALE (0.001)

class GripperDemoIface : public RLLDefaultMoveIfaceBase
{
public:
  explicit GripperDemoIface(const ros::NodeHandle& nh)
    : RLLDefaultMoveIfaceBase(nh)  //, cpa1_(.4, -.25, 0.01, .025), cpa2_(.4, .2, 0.01, .025), cpa3_(0, .55, .01, .025)
  {
    CollisionObjectBuilder builder;

    geometry_msgs::Vector3 bone_bbox_size;
    bone_bbox_size.x = 0.08;
    bone_bbox_size.y = 0.04;
    bone_bbox_size.z = 0.04;

    auto collision = builder.begin()
                         .addMesh("package://rll_move/tests/meshes/bone.dae", MODEL_SCALE)
                         .translate(.2, .3, .01)
                         .build("bone", "world");

    // TODO(mark): the mesh currently has its center on the bottom middle?
    registerGraspObject(std::make_unique<SingleMeshGraspObject>(bone_bbox_size), collision);

    collision = builder.begin()
                    .addBox(.1, .05, .02)
                    .translate(0, .55, 0)
                    .positionBottomAtZ(.02)
                    .rotateRPY(0, 0, M_PI / 4)
                    .build("box1", "world");
    registerGraspObject(std::make_unique<BoxGraspObject>(), collision);

    collision =
        builder.begin().addCylinder(.05, .06).translate(.4, .2, 0).positionBottomAtZ(.06).build("cylinder1", "world");
    registerGraspObject(std::make_unique<CylinderGraspObject>(), collision);
  }

  ~GripperDemoIface() override = default;
};
using GripperDemoSimulationIface = RLLCombinedMoveIface<GripperDemoIface, RLLSimulationMoveIface>;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_demo_iface");
  ros::NodeHandle nh;

  if (waitForMoveGroupAction())
  {
    GripperDemoSimulationIface iface(nh);
    iface.startServicesAndRunNode(&nh);
  }

  return 0;
}
