#include <gtest/gtest.h>
#include <ros/ros.h>

#include <rll_move/permissions.h>

TEST(PermissionsTest, testNoDefaultPermission)
{
  Permissions p;
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));
}

TEST(PermissionsTest, testUpdatePermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  // two permissions are set by default => next index is 2^2
  EXPECT_EQ(p.getCurrentPermissions(), 4u);
  auto p2 = p.registerPermission("p2", false);
  // permissions are unchanged, because p2 is not allowed initially
  EXPECT_EQ(p.getCurrentPermissions(), 4u);
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  EXPECT_TRUE(p.updateCurrentPermissions(p1, false));
  EXPECT_FALSE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  EXPECT_TRUE(p.updateCurrentPermissions(p2, true));
  EXPECT_FALSE(p.isPermitted(p1));
  EXPECT_TRUE(p.isPermitted(p2));
}

TEST(PermissionsTest, testInheritDefaultPermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1");
  p.setDefaultRequiredPermissions(p1);
  // require no explicit permissions, but inherit defaults
  p.setRequiredPermissionsFor("test", Permissions::NO_PERMISSION_REQUIRED, true);
  EXPECT_EQ(p.getRequiredPermissionsFor("test"), p1);
}

TEST(PermissionsTest, testDefaultPermissionBitNotShown)
{
  Permissions p;
  auto p1 = p.registerPermission("p1");
  auto p2 = p.registerPermission("p2");
  p.setDefaultRequiredPermissions(p1 | p2);

  // require no explicit permissions, required defaults
  p.setRequiredPermissionsFor("test", Permissions::NO_PERMISSION_REQUIRED, true);
  auto required = p.getRequiredPermissionsFor("test");
  // even though internally the REQUIRE_DEFAULTS_INDICATOR bit is set, it should not be
  // returned, but rather resolved with the current set permission bits
  auto default_indicator_set = required & Permissions::APPLY_DEFAULTS_BIT;
  EXPECT_EQ(default_indicator_set, 0u);
  EXPECT_EQ(required, p1 | p2);
}

TEST(PermissionsTest, testExplicitPermissionInheritDefaultPermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1");
  auto p2 = p.registerPermission("p2");
  p.setDefaultRequiredPermissions(p1);

  // require explicit permission and inherit defaults
  p.setRequiredPermissionsFor("test", p2, true);
  EXPECT_EQ(p.getRequiredPermissionsFor("test"), p1 | p2);
}

TEST(PermissionsTest, testInheritUpdatedDefaultPermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1");
  auto p2 = p.registerPermission("p2");
  auto p3 = p.registerPermission("p3");

  p.setDefaultRequiredPermissions(p1);
  p.setRequiredPermissionsFor("test", p2, true);
  EXPECT_EQ(p.getRequiredPermissionsFor("test"), p1 | p2);
  // update the default permission -> "test" should now also require p3
  p.setDefaultRequiredPermissions(p1 | p3);
  EXPECT_EQ(p.getRequiredPermissionsFor("test"), p1 | p2 | p3);
}

TEST(PermissionsTest, testUpdateByPermissionName)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", false);
  auto p2 = p.registerPermission("p2", true);

  auto p1_new = p.getIndexForName("p1");
  auto p2_new = p.getIndexForName("p2");
  EXPECT_EQ(p1, p1_new);
  EXPECT_EQ(p2, p2_new);

  EXPECT_FALSE(p.isPermitted(p1));
  p.updateCurrentPermissions("p1", true);
  EXPECT_TRUE(p.isPermitted(p1));

  EXPECT_TRUE(p.isPermitted(p2));
  p.updateCurrentPermissions("p2", false);
  EXPECT_FALSE(p.isPermitted(p2));
}

TEST(PermissionsTest, testUpdateTwoPermissionsAtOnce)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  auto p2 = p.registerPermission("p2", false);
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  EXPECT_TRUE(p.updateCurrentPermissions(p1 | p2, false));
  EXPECT_FALSE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));
  EXPECT_FALSE(p.isPermitted(p1 | p2));

  EXPECT_TRUE(p.updateCurrentPermissions(p1 | p2, true));
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_TRUE(p.isPermitted(p2));
  EXPECT_TRUE(p.isPermitted(p1 | p2));
}

TEST(PermissionsTest, testUpdateInvalidPermission)
{
  Permissions p;
  // try to update the zeroth permission
  EXPECT_FALSE(p.updateCurrentPermissions(0, false));
  auto p1 = p.registerPermission("p1", true);

  // update a permission that has not been registered
  EXPECT_FALSE(p.updateCurrentPermissions(p1 << 1, false));
}

TEST(PermissionsTest, testRegisteredPermissionsValue)
{
  Permissions p;
  // register a permission, test its index, the current permission and
  // if the initial value is set correctly
  auto p1 = p.registerPermission("p1", true);
  EXPECT_EQ(p1, 4u);
  EXPECT_EQ(p.getCurrentPermissions(), p1);
  EXPECT_TRUE(p.isPermitted(p1));

  auto p2 = p.registerPermission("p2", false);
  EXPECT_EQ(p2, 8u);
  EXPECT_EQ(p.getCurrentPermissions(), p1);
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  auto p3 = p.registerPermission("p3", true);
  EXPECT_EQ(p3, 16u);
  EXPECT_EQ(p.getCurrentPermissions(), p1 + p3);
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));
  EXPECT_TRUE(p.isPermitted(p3));
}

TEST(PermissionsTest, testIsPermissionRequired)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  auto p2 = p.registerPermission("p2", false);

  p.setRequiredPermissionsFor("test", p2);
  EXPECT_FALSE(p.isPermissionRequiredFor("test", p1));
  EXPECT_TRUE(p.isPermissionRequiredFor("test", p2));
}

TEST(PermissionsTest, testClearPermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  auto p2 = p.registerPermission("p2", false);
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  p.clearAllPermissions();

  EXPECT_FALSE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));
}

TEST(PermissionsTest, testrestoreSinglePermission)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);

  // store change and restore single permission
  p.storeCurrentPermissions();
  p.updateCurrentPermissions(p1, false);
  EXPECT_FALSE(p.isPermitted(p1));
  p.restorePreviousPermissions();
  EXPECT_TRUE(p.isPermitted(p1));
}

TEST(PermissionsTest, testrestoreMultiplePermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  auto p2 = p.registerPermission("p2", false);
  auto p3 = p.registerPermission("p3", true);

  // store and change permissions
  p.storeCurrentPermissions();
  p.updateCurrentPermissions(p1, false);
  p.updateCurrentPermissions(p2, true);
  p.updateCurrentPermissions(p3, false);
  EXPECT_TRUE(!p.isPermitted(p1));
  EXPECT_FALSE(!p.isPermitted(p2));
  EXPECT_TRUE(!p.isPermitted(p3));

  // restore previous
  p.restorePreviousPermissions();
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));
  EXPECT_TRUE(p.isPermitted(p3));
}

TEST(PermissionsTest, testNestedRestoreMultiplePermissions)
{
  Permissions p;
  auto p1 = p.registerPermission("p1", true);
  auto p2 = p.registerPermission("p2", false);

  // store and change for the first time
  p.storeCurrentPermissions();
  p.updateCurrentPermissions(p1, false);
  EXPECT_TRUE(!p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  // store and change for the second time
  p.storeCurrentPermissions();
  p.updateCurrentPermissions(p2, true);
  EXPECT_TRUE(!p.isPermitted(p1));
  EXPECT_FALSE(!p.isPermitted(p2));

  // restore previous second level
  p.restorePreviousPermissions();
  EXPECT_TRUE(!p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));

  // restore previous first level
  p.restorePreviousPermissions();
  EXPECT_TRUE(p.isPermitted(p1));
  EXPECT_FALSE(p.isPermitted(p2));
}

TEST(PermissionsTest, testSingleDefaultPermission)
{
  Permissions p;
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));
  p.setDefaultRequiredPermissions(Permissions::NO_PERMISSION_REQUIRED);
  EXPECT_TRUE(p.areAllRequiredPermissionsSetFor("test"));

  auto p1_index = p.registerPermission("p1", false);
  p.setDefaultRequiredPermissions(p1_index);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p1_index, true);
  EXPECT_TRUE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p1_index, false);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));
}

TEST(PermissionsTest, testTwoDefaultPermissions)
{
  Permissions p;
  auto p1_index = p.registerPermission("p1", false);
  auto p2_index = p.registerPermission("p2", false);

  p.setDefaultRequiredPermissions(p1_index | p2_index);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p1_index, true);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p1_index, false);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p2_index, true);
  EXPECT_FALSE(p.areAllRequiredPermissionsSetFor("test"));

  p.updateCurrentPermissions(p1_index, true);
  EXPECT_TRUE(p.areAllRequiredPermissionsSetFor("test"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "permissions_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
