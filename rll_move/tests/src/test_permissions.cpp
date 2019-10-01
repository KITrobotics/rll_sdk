#include <ros/ros.h>
#include <gtest/gtest.h>

#include <rll_move/permissions.h>

TEST(PermissionsTest, testNoDefaultPermission)
{
  Permissions p;
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));
}

TEST(PermissionsTest, testSingleDefaultPermission)
{
  Permissions p;
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));
  p.setDefaultRequiredPermissions(Permissions::NO_PERMISSION_REQUIRED);
  EXPECT_TRUE(p.areAllRequirementsMetFor("test"));

  auto p1_index = p.registerPermission("p1", false);
  p.setDefaultRequiredPermissions(p1_index);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p1_index, true);
  EXPECT_TRUE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p1_index, false);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));
}

TEST(PermissionsTest, testTwoDefaultPermissions)
{
  Permissions p;
  auto p1_index = p.registerPermission("p1", false);
  auto p2_index = p.registerPermission("p2", false);

  p.setDefaultRequiredPermissions(p1_index | p2_index);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p1_index, true);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p1_index, false);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p2_index, true);
  EXPECT_FALSE(p.areAllRequirementsMetFor("test"));

  p.updatePermission(p1_index, true);
  EXPECT_TRUE(p.areAllRequirementsMetFor("test"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
