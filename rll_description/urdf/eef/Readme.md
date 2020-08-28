# EEF configuration

You can attach different EEF to the iiwa. The EEF is configured by setting the `eef_type` flag to one of the available options.
You can, however, add your own EEF if your project requires it.

Create the required URDF files, use the included examples in `rll_description/urdf/eef/` as a reference.
Furthermore, add a matching SRDF file, compare with the included examples in `rll_moveit_config/config/`.

See the [Moveit documentation](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html)
for more details on how to create your URDF and SRDF files. 
