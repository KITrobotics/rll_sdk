Robot Playground project
========================

The robot playground project contains Python and C++ examples to help you get
started with the Robot Learning Lab. Follow the steps below to install, build,
run and finally submit your first project!


Installation
------------

If you haven't already setup your RLL workspace with the robot playground
project, do so now by following the instructions outlined in the
:doc:`getting_started` guide.


Building and running the project
--------------------------------

Refer to the :doc:`development_workflow` document for detailed information on
how to build and run the project.
As a quick recap, here are the required commands to build and run the project:

.. code-block:: shell

   catkin build rll_robot_playground_project

To start the simulation environment run:

.. code-block:: shell

      roslaunch rll_robot_playground_project setup_simulation.launch

Run your python code, by executing:

.. code-block:: shell

        roslaunch rll_robot_playground_project run_your_code.launch

And to finally submit your code, run:

.. code-block:: shell

      roslaunch rll_robot_playground_project submit_project.launch

The project files are located in the
:code:`~/rll_ws/src/rll_robot_playground_project` directory.

.. _robot-playground-example:

Hello World Example
-------------------

To demonstrate how to use all the movement methods described in
:doc:`rll_move_client` we will build a full example that utilizes all of them.
The code is extended step by step as more methods are added.
The examples are snippets of the whole program and each example should run on
its own. For each step, the relevant new lines are highlighted.

.. note::
  The example created below is the `hello world` program from the
  `Robot Playground Project <https://gitlab.ipr.kit.edu/rll/rll_robot_playground_project/>`_.

  It is recommended that you follow along and copy the changes as they are added
  into the file :code:`scripts/playground.py` or :code:`src/playground.cpp`, respectively.
  This way you can recreate the example for yourself, run the code after every
  change and get a better understanding of what is happening.


Hello ROS
^^^^^^^^^

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 17,18
         :lines: 1-2, 21-36, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :language: cpp
         :linenos:
         :emphasize-lines: 9,10
         :lines: 20-29, 191-202


We use the :ref:`move-client-getting-started` template and add some simple
logging output. As the comments indicate, you should use the logging methods
provided by ROS, instead of the system default output.


It's moving!
^^^^^^^^^^^^

The previous example didn't actually move the robot. Lets change that!

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 18
         :lines: 1-2, 21-34, 38-43, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :emphasize-lines: 10
         :lines: 20-27, 31-36, 191-202


We use the :ref:`move_random <move-client-move-random>` function to move the
robot into a random position. Adding a delay between movements helps to see
the different movements better.



Specifying joint angles
^^^^^^^^^^^^^^^^^^^^^^^

The next example illustrates how to add error checks, which we previously
neglected.

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 20, 26, 33-34, 42-43, 45-46
         :lines: 1-2, 21-34, 45-76, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :emphasize-lines: 12, 20, 24-27, 34, 36-39
         :lines: 20-27, 38-70, 191-202


We use the :ref:`move_joints<move-client-move-joints>` function to specify
the seven joint positions. As stated in :ref:`move-client-error-handling`,
service calls return :code:`False` on failure. In this example the second call
:code:`move_client.move_joints(0.0, 0.0, 0.0, pi / 2, 0.0, 0.0, 0.0)` would
move the robot outside the allowed workspace. As a result the call fails and no
movement is made. Without the error check you might not have noticed that
the call failed!


.. hint::
   If you check the output log you will notice that the failure has been reported there,
   too. Service call failures will always be logged, regardless of your own error checks.

More movement
^^^^^^^^^^^^^

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 19-21, 24, 39, 41, 46, 48
         :lines: 1-2, 21-34, 78-111, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :emphasize-lines: 13-17, 20, 36, 40, 45, 47
         :lines: 20-27, 48, 71-110, 191-202

Point based movement ist easier to understand. You only need to specify
the position and orientation of the end effector and the robot will move there.
However, you cannot control how the robot gets to this position.

The orientation part of a pose is stored as a quaternion. You usually don't set
the quaternion manually. Use the provided helper functions to generate the
quaternion for you.



Linear movement
^^^^^^^^^^^^^^^

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 23, 26, 29, 32, 36, 39, 41, 44-45, 47
         :lines: 1-2, 21-34, 80, 118-153, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :emphasize-lines: 15, 18-20, 23, 26, 30, 33, 36, 39, 40, 43
         :lines: 20-27, 74, 116-149, 191-202


Previously we moved in a point to point fashion to a desired pose.
However, this has the drawback that we cannot guarantee how the end effector
is reaching its new position. One alternative is to use
:ref:`move-client-move-lin` instead. This ensures that the end effector is
moved on a linear trajectory to its target position. In the example above,
three consecutive linear movements, forming a triangular path, are executed.

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :emphasize-lines: 26, 33, 28, 35
         :lines: 1-2, 21-34, 80, 100, 126, 148-166, 184-195

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :emphasize-lines: 22, 31, 24, 33
         :lines: 20-27, 48, 74, 97, 124-126, 150-171, 191-202

Since :code:`move_lin()` requires the end effector to travel on a
linear trajectory, it is more constrained than :code:`move_ptp()`
which imposes no restrictions on how to reach the goal pose.
It is therefore possible that :code:`move_lin()` fails where a movement with
:code:`move_ptp()` succeeds, as illustrated in the example above.


.. _robot-playground-complete:

Complete example
^^^^^^^^^^^^^^^^

The complete code, which encompasses the examples above, is shown below:

.. tabs::

   .. group-tab:: Python

      .. literalinclude:: _static/code_examples/hello_world.py
         :linenos:
         :caption: Complete hello_world.py example
         :lines: 1-2, 21-

   .. group-tab:: C++

      .. literalinclude:: _static/code_examples/hello_world.cpp
         :linenos:
         :language: cpp
         :caption: Complete hello_world.cpp example
         :lines: 20-
