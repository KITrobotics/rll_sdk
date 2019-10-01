RLL Virtual Machine
===================

If you don't want to install the RLL environment manually you can use the provided `VirtualBox <https://virtualbox.org>`_ VM. The VM contains a preconfigured development environment
and is based on Lubuntu 18.04 with ROS Melodic Morenia.


Installation
--------------

1. Download the RLL VM packaged as an `.ova` file `here <todo.de>`_.

.. todo::
   Download link

2. Start VirtualBox and import the VM by choosing `File->Import Appliance`.

3. Select the downloaded `.ova` file and adjust the machine settings if
   required.

4. Click `Import`, this will take a while.

5. By default the VM has only 8MB of `Video Memory`. It is highly recommended
   to increase that value. Open the VM settings, go to the `Display Settings`
   and increase the `Video Memory` e.g. up to `128MB`. You can also tick the
   `Enable 3D Acceleration` checkbox. If you later notice that the VM crashes
   or has display issues un-tick the checkbox.

6. If you have completed these steps VM is setup and can be powered on.

.. note:: The default username and password are both: **rll**

.. note:: The VM is configured to use a German keyboard layout by default.


Using the VM
------------

The WM contains an already initialized catkin workspace in
:code:`~/rll_ws` with the :doc:`Robot Playground project <project_robot_playground>`
set up and ready to run. The VM comes with some handy desktop shortcuts
to make launching and editing the robot playground project more convenient.

.. figure:: _static/rll_vm_desktop.png
    :align: center
    :figclass: align-center

    The VM with desktop shortcuts on the left to edit, run and submit the robot playground project.


Desktop shortcuts
^^^^^^^^^^^^^^^^^

You can use the provided desktop shortcuts to build, edit and submit the
robot playground project instead of running the commands manually as described
in the :doc:`documentation <project_robot_playground>`.
The shortcuts are convenient but your development workflow will probably be
faster if you execute the commands in a terminal.

1. **Start the Robot Playground** This will build the project, start the
   simulation and run your code as described in :ref:`playground-run`.

   .. note:: This will open a Tilix terminal displaying the log output and Rviz showing the robot visualization.

2. **Edit the Robot Playground** Launches the VS Code editor with the source
   code of the robot playground already opened.

3 **Submit the Robot Playground** uploads your code to be run on a real robot.

  .. note:: You still need to setup your API access token first as described in :ref:`making-your-first-submission`


Editing files
^^^^^^^^^^^^^

The VM comes with `VS Code <https://code.visualstudio.com/>`_
and `Eclipse <https://www.eclipse.org/>`_ installed.
VS Code is used as the default editor in the tutorial and is preconfigured
with some useful settings.

.. hint:: You can, of course, use any other text editor of your choice.
          `Gedit`, for example, is an alternative lightweight text editor.


Debugging your Code with VS Code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A :code:`launch.json` config is provided so you can debug your code right away.
For more information on debugging consult the `VS Code documentation <https://code.visualstudio.com/docs/editor/debugging>`_. To start the project with the VS Code debugger execute the following steps:

1. **Start the simulation environment**: Either run
   :code:`roslaunch rll_robot_playground_project setup_simulation.launch` in a
   terminal or use the provided VS Code task. The task can be executed by
   opening the command pallette (`Ctrl+Shift+P`) typing `Run Task` and
   choose `RLL: Setup playground simulation`.

2. Choose either one of the following options:

   1. **Launch your Python code**: Open the :code:`playground.py` file in the
      editor and place breakpoints if you wish. Go to the debug perspective
      (`Ctrl + Shift + D`). Make sure to select the configuration
      `Python: Debug playground.py` and start debugging.

   2. **Launch your C++ code**: Open the :code:`playground.cpp` file in the
      editor and place breakpoints if you wish. Go to the debug perspective
      (`Ctrl + Shift + D`). Make sure to select the configuration
      `C++: Debug playground.cpp` and start debugging. Don't forget to build
      your code first, e.g. by running the `RLL: Build C++ Code` task.

3. **Trigger the project execution**: Run
   :code:`roslaunch rll_tools run_project.launch` in a terminal or use the
   provided VS Code task `RLL: Trigger project execution`.

.. hint::
   If you run the project code directly e.g. in a terminal or other IDE you need to
   set the environment variable :code:`ROS_NAMESPACE=iiwa` before you launch your
   code. The VS Code :code:`launch.json` configuration already does this for you.
