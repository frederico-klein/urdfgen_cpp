# urdfgen_cpp

Port of python addin. 

Currently in a beta state. Some bugs remain.

## Known issues:

- Sometimes occurrences origins are not parsed correctly and the link is incorrectly placed. Workaround is to place all derived Occurrences instead of the original one (more clicking).

- For some reason the selected joint is not highlighted when working with a UJoint control. 

- ULink and UJoint names are not editable.

- Deleting all Links places a link that does not have the name base, which will make it impossible for "Create tree" to find base.

For new issues/bug reports please open ticket under Issues.
