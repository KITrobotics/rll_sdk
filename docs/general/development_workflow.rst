Development workflow
====================

At this point you should have already setup your RLL workspace and
followed the steps from the :doc:`getting_started` guide.


The instructions in the document will help you build and run your project. The
commands which are described here can be applied to any project that is
available with the RLL. However, we will use the
:doc:`project_robot_playground` as an example project and you need to
replace the project name in the project specific commands to adapt
it to different projects. The project specific commands are all of the format:
:code:`roslaunch <project-name> <arguments>`

General notes
-------------

The following commands need to be execute in a terminal window, i.e. copy and
paste or type them in the terminal prompt and press `Enter` to execute them.
While a command is running you will see the command output and cannot
execute any new commands in this terminal until the command is finished or you
stop it.

1. Any command you should execute will be displayed like this:

.. code-block:: shell

   <command> <argument1> <argument2>

2. Commands need to be run in a terminal where the ROS environment has been
initialized. To initialize the the ROS environment execute the following
command:

.. code-block:: shell

   source ~/rll_ws/devel/setup.bash

You need to do this with **every** new terminal you open! To avoid having to
do this every time you start a terminal it is highly recommended to add the
above command at the end of your `~/.bashrc` file.

.. note:: If you are using the RLL VM the `~/.bashrc` file has already been adjusted for you and you do not need to do this.

3. Commands should be run from within your catkin workspace directory
   :code:`~/rll_ws`. To navigate in to this directory execute:

   .. code-block:: shell

      cd ~/rll_ws

   If you use a different workspace folder other than `~/rll_ws`, adjust the
   path in the above command accordingly.


Build the project
-----------------

Make sure you are in your catkin workspace directory and your ROS environment
is initialized. Run the following command to build the project:

.. code-block:: shell

   catkin build rll_robot_playground_project

If you are going to write your code in Python you only need to build
the project **once**. If you are using C++ you will need to run this command
every time you make changes to your code.

.. _playground-run:

Run the project
---------------

During development you run your code in the RLL simulation environment.
Therefore, you first need to start simulation and then run your
code. Execute each command in a **new** terminal tab or window.

1. First, Start the simulation environment:

   .. code-block:: shell

      roslaunch rll_robot_playground_project setup_simulation.launch

   The terminal will display some log output and will continue to print logging
   information while you run your code. Additionally, `RViz` is opened showing
   the robot visualization. You should already be able to see the robot move
   into its *home* position.

2. Second, execute your project code. Depending on whether you want to
   use Python or C++ run **one** of the following commands:

   - Python version (the default):

     .. code-block:: shell

        roslaunch rll_robot_playground_project run_your_code.launch

   - C++ version:

     .. code-block:: shell

        roslaunch rll_robot_playground_project run_your_code.launch use_python:=false

   The robot should now begin to move and the log output of the program
   is printed to the terminal.


To execute the project code again, interrupt the second command you executed
and simply re-run it. To interrupt the running command press :code:`Ctrl + C`
in the corresponding terminal.

.. hint:: If something goes wrong or you close a terminal by accident, it is best to close all terminals that are still open. Once all windows are closed repeat all the steps above.


Write your own code
-------------------

Once you are able to start the simulation and execute the project code you
can go ahead and start to modify the project and write your own code.

Usually the project code is included in two versions, one version written in
Python and one in C++. The C++ code can be found in the :code:`src/` folder
whereas the Python code is located in the :code:`scripts/` directory within
the project folder.

The full path to the project folder is: :code:`~/rll_ws/src/<project-name>`
I.e. for the Robot playground project it is
:code:`~/rll_ws/src/rll_robot_playground_project`.

Where to put your code
^^^^^^^^^^^^^^^^^^^^^^

In case of the Robot playground project the code in the file
:code:`scripts/playground.py` or :code:`src/playground.cpp`, respectively,
is executed. Therefore place your own code in these files.

Depending on the project there will be multiple files, in case of the Robot
playground project there are the files :code:`scripts/hello_world.py`
and :code:`src/hello_world.cpp`, respectively. These files contain the same
code as the project's main code file.
This way you can initially run the project without having to change anything.
Feel free to delete and modify the code. You can always use the *hello_world*
file as a reference or to restore its original content.


Re-Build your code
^^^^^^^^^^^^^^^^^^

As stated above, if you are using C++ you need to re-build your code every time
you made changes to the code and want to test them. Therefore, once again run:

.. code-block:: shell

   catkin build rll_robot_playground_project


Run your code
^^^^^^^^^^^^^

You can run your code by following the steps from *"Run the project"* as
shown above.
To run your modified code you need to restart the command from step 2. To do
this, interrupt the running command by pressing :code:`Ctrl + C` in the
terminal in which you executed the command from step 2. Once the command
has stopped simply execute the command again.

.. hint:: If you are unsure about this it is best to close all terminal windows
   and repeat the steps 1 and 2.


.. _make-c++-default:

Use C++ by default
^^^^^^^^^^^^^^^^^^

If you are programming in C++ you can and should adjust the launch files to use
C++ by default.

1. Open the file :code:`launch/move_sender.launch`, which is located in the
   project folder and change the following line, while keeping the remaining
   lines as they are, from:

   .. code-block:: xml

      <arg name="use_python" default="true" />

   to:

   .. code-block:: xml

      <arg name="use_python" default="false" />

2. Repeat step 1 but this time modify the file
   :code:`launch/run_your_code.launch`

If you run your code now as described in :ref:`playground-run`, the file
:code:`playground.cpp` is executed by default.

.. warning:: You will need to make this change if you want to submit your C++
   code! On the real robot the launch file is executed without overriding
   the default.


Submit your code
^^^^^^^^^^^^^^^^

To run your code on a real robot, follow the steps as described in
:ref:`configure-api-access`. That is run the following command:

.. code-block:: shell

      roslaunch rll_robot_playground_project submit_project.launch

