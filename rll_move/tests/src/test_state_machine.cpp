#include <ros/ros.h>
#include <gtest/gtest.h>

#include <rll_move/move_iface_state_machine.h>

TEST(PermissionsTest, testBeginServiceCall)
{
  RLLMoveIfaceStateMachine s;
  auto resp = s.beginServiceCall("test");
  ASSERT_EQ(RLLErrorCode::SERVICE_CALL_NOT_ALLOWED, resp.value());

  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SERVICE_CALL_NOT_ALLOWED);

  bool success = s.enterState(RLLMoveIfaceState::RUNNING_JOB);
  ASSERT_TRUE(success);

  resp = s.beginServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.beginServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::CONCURRENT_SERVICE_CALL);
  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);
}

TEST(PermissionsTest, testEnterState)
{
  RLLMoveIfaceStateMachine s;
  auto success = s.enterState(RLLMoveIfaceState::RUNNING_JOB);
  ASSERT_TRUE(success);

  success = s.enterState(RLLMoveIfaceState::IDLING);
  ASSERT_FALSE(success);

  ASSERT_TRUE(s.isInInternalErrorState());
}

TEST(PermissionsTest, testLeaveErrorState)
{
  RLLMoveIfaceStateMachine s;
  // enter error state
  auto success = s.enterState(RLLMoveIfaceState::RUNNING_JOB);
  success = s.enterState(RLLMoveIfaceState::IDLING);
  ASSERT_TRUE(s.isInInternalErrorState());

  // try to change state
  success = s.enterState(RLLMoveIfaceState::WAITING);
  ASSERT_FALSE(success);
  ASSERT_TRUE(s.isInInternalErrorState());

  success = s.enterState(RLLMoveIfaceState::RUNNING_JOB);
  ASSERT_FALSE(success);
  ASSERT_TRUE(s.isInInternalErrorState());

  success = s.enterState(RLLMoveIfaceState::IDLING);
  ASSERT_FALSE(success);
  ASSERT_TRUE(s.isInInternalErrorState());
}

TEST(PermissionsTest, testServiceAllowedOutsideJobRunCall)
{
  RLLMoveIfaceStateMachine s;

  auto resp = s.beginServiceCall("test");
  ASSERT_EQ(RLLErrorCode::SERVICE_CALL_NOT_ALLOWED, resp.value());

  resp = s.endServiceCall("test");
  ASSERT_EQ(RLLErrorCode::SERVICE_CALL_NOT_ALLOWED, resp.value());

  // allow outside of job run (i.e. in waiting)
  resp = s.beginServiceCall("test", false);
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.endServiceCall("test", false);
  ASSERT_EQ(RLLErrorCode::SUCCESS, resp.value());

  // idle state should fail
  s.enterState(RLLMoveIfaceState::IDLING);
  resp = s.beginServiceCall("test", false);
  ASSERT_EQ(RLLErrorCode::SERVICE_CALL_NOT_ALLOWED, resp.value());

  resp = s.endServiceCall("test", false);
  ASSERT_EQ(RLLErrorCode::SERVICE_CALL_NOT_ALLOWED, resp.value());

  // error state should fail as well
  s.enterErrorState();
  resp = s.beginServiceCall("test", false);
  ASSERT_EQ(RLLErrorCode::INTERNAL_ERROR, resp.value());

  resp = s.endServiceCall("test", false);
  ASSERT_EQ(RLLErrorCode::INTERNAL_ERROR, resp.value());
}

TEST(PermissionsTest, testFullDemoRun)
{
  RLLMoveIfaceStateMachine s;

  auto success = s.enterState(RLLMoveIfaceState::RUNNING_JOB);
  ASSERT_TRUE(success);

  auto resp = s.beginServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);
  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.beginServiceCall("test1");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);
  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.beginServiceCall("test2");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);
  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  resp = s.beginServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);
  resp = s.endServiceCall("test");
  ASSERT_EQ(resp.value(), RLLErrorCode::SUCCESS);

  success = s.leaveState();
  ASSERT_TRUE(success);

  success = s.enterState(RLLMoveIfaceState::IDLING);
  ASSERT_TRUE(success);

  success = s.leaveState();
  ASSERT_TRUE(success);
}
