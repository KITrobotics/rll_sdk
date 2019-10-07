Getting started
===============

The first step is to register yourself. You can do this `here <https://rll.ipr.kit.edu/login>`_.
We support regular user registrations using your Email address or you can log in with a Google or Github account.
If you are a student or employee at KIT, you can directly log in with your KIT account.

Running demos
-------------

To get familiar with the submission processing in the lab, you can
`test some of the demos <https://rll.ipr.kit.edu/demos>`_.
These are example implementations for projects that are currently hosted in the lab
or that will become available at a later point. Once you submitted one or several demos, you can follow the processing
of your jobs in the `jobs list view <https://rll.ipr.kit.edu/jobs>`_. There you can view a live feed of the webcam while
the job is running and inspect job data, including log files once the job has finished.

The :doc:`job processing chapter <job_processing>` explains how your submission is handled by the lab system
and what the different job states mean.

.. _making-your-first-submission:

Making your first submission
----------------------------

1. First you need to set up your development environment. You can either do a
   :doc:`manual installation <manual_installation>` of all required packages or start with our
   :doc:`ready-to-use virtual machine image <vm_setup>`.

   We recommend the manual installation if you already have an Ubuntu or Debian system on your machine or if you are
   interested in setting up an Ubuntu installation by yourself. Otherwise the VM image is easier to get started
   and it works on Windows and macOS.

.. hint::
   Additonally to these installation options, a web-based development environment is currently in development
   and will be offered as an alternative once it is ready.

2. Download the API access config from the `settings page <https://rll.ipr.kit.edu/settings>`_
   and save it to the config folder of the ``rll_tools`` package in your workspace.
   This should be the full path to the file:

   .. code-block:: shell

      ~/rll_ws/src/rll_sdk/rll_tools/config/api-access.yaml

   You only need to do this step once. The config contains an access token that you can use for an arbitrary
   number of submissions.

3. Now you can make a submission by running:

   .. code-block:: shell

      roslaunch rll_robot_playground_project submit_project.launch

   This command will create an archive of your source code for the Robot Playground project and upload it
   to the Robot Learning Lab API. If you did not yet make any changes to the source code, then it will
   simply upload the default hello world program. You can then follow the job in your
   `jobs view <https://rll.ipr.kit.edu/jobs>`_.

   You can execute this command anytime you want to see your current code version running on one of
   the robots in the lab.

Writing your own code
---------------------

The :doc:`Robot Playground project <project_robot_playground>` is the starter project that lets you move
the robot around using different commands. You can use this project to get familiar with the development environment
and the different ways to control the robot. Follow the :doc:`project guide <project_robot_playground>`
to get started.
