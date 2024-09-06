# urdfgen_cpp

This is the source code for the Fusion360 ROS URDF package generator. 

It allows you to create the URDF model definition and meshes without having to export it intermediately to solidworks and redefining the origins and joints.


This is a port of python addin from [urdf addin from GuimmiArmCE's fusion360 scripts](https://github.com/GummiArmCE/fusion360_scripts/blob/master/urdf_generator/urdf-addin.py). 

It was fairly functional last time I have checked, if something isn't working, please open an issue. Some "minor" bugs still remain (see below), please let me know if it bothers you and I will have a crack at it.

## Known issues:

- Sometimes occurrences origins are not parsed correctly and the link is incorrectly placed. Workaround is to place all derived Occurrences instead of the original one (more clicking).

- For some reason the selected joint is not highlighted when working with a UJoint control. 

- ULink and UJoint names are not editable.

- Deleting all Links places a link that does not have the name base, which will make it impossible for "Create tree" to find base.

For new issues/bug reports please open ticket under Issues.
