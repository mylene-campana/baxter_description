# baxter_description

This package containts a problem for motion planning: a baxter-like robot 
that should move in an environment. For example in the script, the robot is 
manipulation in a room.

The package contains:

  - URDF/SRDF/class files describing the objects,
 
  - Some Python scripts going along with HPP software (github.com/humanoid-path-planner) for motion planning,

The problem can be vizualised with HPP-gepetto-viewer (github.com/humanoid-path-planner) 
or with RViz (must create .launch files).

To install the package with cmake, simply:

  - Create a 'build' directory in the source package,
  
  - in the created /build, configure the package - particularly the 'install path variable' - and install it 
  with 'ccmake ..' and 'make install'.
