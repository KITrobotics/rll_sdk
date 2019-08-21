Job processing
==============

Every submission is a job. When you submit a demo or upload your own code,
the submission is inserted into the job queue of a specific project.
The lab will offer different projects that have their own experimental setup
in the robot work cells and their own software package you can work with.
The :doc:`Robot Playground project <project_robot_playground>` is such a project.

This chapter gives an overview of the job queue, the job metadata and explains
how submitted code is executed. For more in-depth information,
see our :doc:`publications <publications>`.

Job queue
---------

The jobs are processed using a `priority queue <https://en.wikipedia.org/wiki/Priority_queue>`_.
Generally for every project, the jobs are queued using the first in, first out method.
However different priorities apply. Submissions for demo projects have a lower
priority than regular submissions of user code. So every regular submission in the queue is
processed before the oldest demo submission.

Demo submissions run directly on the robots, while regular submissions are first tested in
simulation. If the simulation check does not detect serious errors, then the job is inserted
into the job queue for execution on the real robots.

Every user can have at least one job per project in the job queue. If more than one robot in the lab
is currently processing a certain project, then as many submissions as robots are online
for the particular project are allowed for every user.

As long as a job has the status `submitted` and processing did not yet start,
updates are still allowed. This means that if you have updated your code in the meantime
and submit again, the job in the queue will be updated with your newly submitted code and the
old code version will not be used anymore.

Execution environment
---------------------

Your submitted code is executed in `Docker containers <https://www.docker.com/resources/what-container>`_.
The containers has the same ROS installation as your :doc:`local setup <manual_installation>`.
The ROS distro you use is automatically detected when submitting and the corresponding container is used.
So if you submit from a ROS Melodic system,
then your code will be executed in a container with a ROS Melodic installation.

When your code is executed by the lab system, it will have access to the exact same interfaces
as in your local development environment. This also means that the same move interface to control
the robot is used for the simulation and for the real robot. Hence your code does not require
any changes to run on the real robot.

In the execution environment, you have access to your very own
`ROS Master <http://wiki.ros.org/ROS/Concepts#ROS_Computation_Graph_Level>`_.
This allows you to start additional ROS nodes, register your own ROS services, topics or actions,
or set new ROS parameters.

Job metadata
------------

Every job has a unique ID, a job ID. This job ID is used to track the job in the system.
Different job status and result codes are used to clarify the current state of a job
in the `jobs list <https://rll.ipr.kit.edu/jobs>`_. They are explained below.

When the job has either the status `submitted` or `waiting for real`,
the position of the job in the job queue is additionally displayed.

Job status codes
^^^^^^^^^^^^^^^^

The status codes describe the current job state in the queue.

submitted
  The submission was successfully inserted into the job queue.

processing sim started
  The job is the next in line and was selected for processing in simulation.

processing real started
  The job is the next in line and was selected for processing by the real robot.

building
  The submitted code is compiling.

running sim
  The job is currently executed in simulation.

waiting for real
  The job successfully passed the simulation check and is now waiting for execution with the real robot.

running real
  The job is currently executed by the real robot.

finished
  The processing of this job is finished.

Job result codes
^^^^^^^^^^^^^^^^

The result codes are used to describe the result of a job after it was executed in simulation or with the real robot.

reading project archive failed
  The submitted code archive could not be unpacked.

building project failed
  Compiling the submitted project resulted in an error.

real failure
  The submitted code produced an error during execution with the real robot.

sim failure
  The submitted code produced an error during execution in simulation.

real internal error
  An internal error happened during job execution with the real robot.
  The error was not caused by the submitted user code, but by the lab system.

sim internal error
  An internal error happened during job execution in simulation.
  The error was not caused by the submitted user code, but by the simulation system.

real success
  Submission was successfully executed with the real robot.

sim success
  Submission was successfully executed in simulation.
