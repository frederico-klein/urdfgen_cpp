# urdfgen_cpp

Port of python addin. 

Work in progress.
Mostly done. Fixed some bugs from the python version that were easier to spot with a cleaner implementation. 


## todo:

- The attempted dictionary like implementation of urdftree did not work and uses a bunch of workarounds. Either should implement a proper dictionary or skip it completely
- there is no control to fix a joint that has improper origin. This should never be necessary (it's up to the API to give me valid joint pointers), but if it does, the sixdegre class needs to be implemented as well. 
- Currently is not generating stls for links. 
