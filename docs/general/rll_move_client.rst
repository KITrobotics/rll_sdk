RLL MoveClient
==============
.. _rll-move-client:

The robots in the Robot Learning Lab can be interfaced with through the
`RLL MoveIface`. This interface provides several ROS services which can
be called to move the robot. To make the interaction simpler the
:code:`RLL MoveClient` library is provided that unifies the process of calling
the :code:`RLL MoveIface`. This allows you to focus on implementing your own
logic instead of having to deal with ROS specifics.

In this document the available methods on how to use the `MoveClient` are
introduced by providing short code examples.

.. hint:: This section introduces the available commands by using short
   code examples. If you want to get started directly with a complete example
   check out the Robot playground project :ref:`robot-playground-example`.
   It uses all the concepts that are introduced in this section.

.. _move-client-getting-started:

Getting started
---------------

The following examples are provided for Python and C++. Both languages are
equally well supported and you can choose to use either. However, Python may
be a better choice for beginners.
If references to code examples are made, they refer to the Python code by
default, but it should be obvious what part of the C++ code this relates to.

We will modify the :doc:`project_robot_playground` code and
you should already be familiar with the :doc:`development_workflow` to be able
to run the code examples.

To get started, copy and paste the code below into the file
:code:`scripts/playground.py` or :code:`src/playground.cpp`, respectively.
This code is required to interact with the :code:`RLL MoveIface` and will
serve as a starting point for the following examples.

.. tabs::

   .. code-tab:: py

      #! /usr/bin/env python

      from math import pi

      import rospy
      from geometry_msgs.msg import Pose, Point

      from rll_move_client.client import RLLDefaultMoveClient
      from rll_move_client.util import orientation_from_rpy


      def hello_world(move_client):
          rospy.loginfo("Hello world")

          # put your code here

          return True


      def main():
          rospy.init_node('hello_world')
          client = RLLDefaultMoveClient(hello_world)
          client.spin()


      if __name__ == "__main__":
          main()

   .. code-tab:: c++

      #include <ros/ros.h>
      #include <geometry_msgs/Pose.h>

      #include <rll_move_client/move_client_default.h>
      #include <rll_move_client/util.h>

      bool helloWorld(RLLDefaultMoveClient* const move_client)
      {
        ROS_INFO("Hello world");

        // put your code here

        return true;
      }

      int main(int argc, char** argv)
      {
        ros::init(argc, argv, "hello_world");
        RLLCallbackMoveClient<RLLDefaultMoveClient> client(&helloWorld);
        client.spin();
        return 0;
      }

Internally, the `RLL MoveClient` creates a socket listener, which, once a
start signal is received, calls the specified callback function, in this case
:code:`hello_world()`.

.. note::
  The code snippets shown below need to be inserted below
  the comment :code:`# put your code here`.


.. workspace:

Robot workspace
---------------

Before you start writing your own movement code it is helpful to know which
positions the robot can actually reach.

The robot is mounted on a table, where the table defines the workspace
boundaries in the x- and y-direction. In the figure below you can see a
schematic overview of the default setup. The origin is in the middle of the
table and the robot is mounted `0.2m` behind it. The positive x-axis is
pointing to the front, the positive y-axis is oriented to the right and the
z-axis is pointing upwards.

.. _robot-workspace:
.. figure:: _static/robot_workspace.svg
   :align: center
   :figclass: align-center

   Schematic view of the robot workspace. All dimensions are given in meters.

Depending on the current project there may be additional object fixed to the
table.

.. _move-client-move-ptp:

Point to point movement
-----------------------

The easiest way to move the robot's end effector to a user defined pose is to
call the :code:`move_ptp` service. The service requires a :code:`Pose` argument
which holds the target position and orientation.

.. tabs::

   .. code-tab:: py

      goal_pose = Pose()
      goal_pose.position = Point(.5, .2, .7)
      # rotate 90 degrees around the y axis
      goal_pose.orientation = orientation_from_rpy(0, pi / 2, 0)

      move_client.move_ptp(goal_pose)

   .. code-tab:: c++

      geometry_msgs::Pose goal_pose;
      goal_pose.position.x = .5;
      goal_pose.position.y = .2;
      goal_pose.position.z = .7;
      // rotate 90 degrees around the y axis
      orientationFromRPY(0, M_PI / 2, 0, &goal_pose.orientation);

      move_client->movePTP(goal_pose)

.. _move-client-move-joints:

Specifying joint angles
-----------------------

You can also specify the robot's joint angles directly by using the
:code:`move_joints` service. Joint angles are specified in radians and you
can either pass them as seven separate values or as a list of joint values.


.. tabs::

   .. code-tab:: py

      # specify each joint angle separately
      move_client.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

      # or pass a list of joint angles
      joint_values = [pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      move_client.move_joints(joint_values)

   .. code-tab:: c++

      // specify each joint angle separately
      move_client->moveJoints(0, 0, 0, 0, 0, 0, 0);

      // or pass a vector of joint angles
      std::vector<double> joint_values{ M_PI / 2, 0, 0, 0, 0, 0, 0 };
      move_client->moveJoints(joint_values);



The different joints have different lower and upper joint angle limits. Setting
the joint values to zero is the initial configuration of the robot.
In the table :ref:`below<joint-limits>` you can see the lower and upper limits
specified in radians.

.. _joint-limits:

.. table:: The lower and upper joint angle limits specified in radians as multiples of π.

   =========  ===========  ===========
   Joint #    Lower limit  Upper limit
   =========  ===========  ===========
   1          -0.938 π     0.938 π
   2          -0.661 π     0.661 π
   3          -0.938 π     0.938 π
   4          -0.661 π     0.661 π
   5          -0.938 π     0.938 π
   6          -0.661 π     0.661 π
   7          -0.966 π     0.966 π
   =========  ===========  ===========



.. hint:: These are the maximum joint angles, the actual limits you
   can reach are a bit lower.


.. _move-client-move-lin:

Linear movement
---------------

If you want to move the end effector on a linear trajectory, starting at
the current pose, call the :code:`move_lin` service.

.. tabs::

   .. code-tab:: py

      goal_pose = Pose()
      # set position and orientation of the pose...

      move_client.move_lin(goal_pose)

   .. code-tab:: c++

      geometry_msgs::Pose goal_pose;
      // set position and orientation of the pose...

      move_client->moveLin(goal_pose);


.. hint::
  A linear movement is more constrained than a PTP movement and may fail where
  a PTP movement succeeds.

.. _move-client-move-random:

Random movement
---------------

You can move the robot into a random position by calling the
:code:`move_random` service. This is a good start if you just want to see
the robot move.

.. tabs::

   .. code-tab:: py

      move_client.move_random()

   .. code-tab:: c++

      move_client->moveRandom();


If you want to know where the robot has moved to, you can retrieve the
chosen random pose:

.. tabs::

   .. code-tab:: py

      # returns the chosen random pose
      pose = move_client.move_random()

   .. code-tab:: c++

      geometry_msgs::Pose pose;
      # store the pose in the pointed to Pose object
      move_client->moveRandom(&pose);

.. _move-client-arm-angle:

Controlling the arm angle
-------------------------

The robots in the lab have seven degrees of freedom, while only six degrees of freedom are required to define the end effector pose. The additional degree of freedom can be controlled using an arm angle. The arm angle defines the positon of the elbow on a circle with the line connecting shoulder and wrist as rotation axis. The angle should be between -pi and pi.

For point to point movements and linear paths, a goal arm angle can be specified.

.. tabs::

   .. code-tab:: py

      goal_pose = Pose()
      # set position and orientation of the pose...

      goal_arm_angle = pi / 2

      move_client.move_lin_armangle(goal_pose, goal_arm_angle)
      # or
      move_client.move_ptp_armangle(goal_pose, goal_arm_angle)

   .. code-tab:: c++

      geometry_msgs::Pose goal_pose;
      // set position and orientation of the pose...

      double goal_arm_angle = M_PI / 2;

      move_client->moveLinArmangle(goal_pose, goal_arm_angle);
      // or
      move_client->movePTPArmangle(goal_pose, goal_arm_angle);

.. _move-client-error-handling:

Error handling
--------------

There are various reasons why a service call might fail:

- by passing invalid values e.g. joint angles outside the allowed range
- requesting a linear motion to a goal pose, but the robot cannot move
  to this goal pose on a straight line.
- some other unforeseen reason

You will not know in advance if your movement request is successful.
Therefore, it is important to validate the success of a service call.

.. hint::
   If something went wrong and you want to know why, consult the output log. The `Rll MoveClient`
   by default is rather verbose and provides a detailed output of the requests made and
   responses received.


.. tabs::

   .. code-tab:: py

      response = move_client.move_random()
      if not response:
         rospy.loginfo("Service call failed!")

   .. code-tab:: c++

      geometry_msgs::Pose pose;
      bool success = move_client->moveRandom(&pose);
      if (!success){
         ROS_INFO("Service call failed!");
      }


The return values of services calls indicate the success of an invocation. Here
the Python and C++ version differ slightly. Some services calls in Python may
provide a return value other than a boolean. E.g. :code:`move_random` returns
a :code:`Pose` on success or :code:`None` on failure instead.
However due to the `truthy/falsy` behavior of the return values, you can
still test them the same way as if they were boolean values.


Service calls and exceptions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a critical error is reported by the `RLL MoveIface`, the `RLL MoveClient`
throws an exception and aborts your code. This is done because if something
fails in a critical manner, the `RLL MoveIface` aborts all operations, and will
no longer accept new movement requests.

However, this is only the case for critical failures, e.g. the robot detects
a deviation from its expected position. For non-critical failures, such as
trying to move to an unreachable pose, no exception is raised and only the
result of the service call indicates a failure.

If you want to throw an exception on any failure, critical or not, you can
configure the `RLL MoveClient` to do so.

.. tabs::

   .. code-tab:: py

      # raise an exception on any service call failure regardless of severity
      move_client.set_exception_on_any_failure(True)

   .. code-tab:: c++

      // raise an exception on any service call failure regardless of severity
      move_client->setExceptionOnAnyFailure(true);


This is particularly useful if you do not want to concern yourself with error
checks. E.g. your application is *all or nothing*, meaning if any service call
fails, abort the whole program.
