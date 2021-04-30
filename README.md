# Robot Arm Library

This library was developed by NDSU CSCI 445 students in the capstone class in the Spring 2021 semester for John Deere Electronics Solutions.

## Installation

This library requires at least Python Version 3, and is OS Independent without the use of the Orocos KDL library.

1. Begin by cloning this repository inside your workspace
2. *cd* into the directory
3. Depending on your needs, execute one of the following commands:
    * To simply install to Python run: `pip3 install .`
    * To install in developer mode run: `pip3 install -e .` (developer mode sets a link to this directory, and allows for a live working copy to be used)
4. Depending on your current installation of Python you may have to install the C library dependencies for the SciPy package.

**If you are using Orocos KDL Library:** currently OrocosKDL has a wrapper that is not fully implemented and working with URDF. If you are on a Unix OS follow the instructions here `github.com/orocos/orocos_kinematics_dynamics` to download and install the Orocos Kinematics and Dynamics Library (**Note:** This installation assumes you using a Debian/Ubuntu OS!).

## Features

* Moves robotic arm to XYZ position in cartesian space
* Enables arm to open and close hand around object
* Enables arm to rotate hand for proper orientation of object
* Calculates forward and reverse kinematics of arm
    * Utilizes multiple wrappers to allow various kinematics solver libraries
* Utilizes URDF format to describe robot in internal files
* Provides visual virtualization of instructed arm movement using matplotlib

## Usage

### URDF Files

This library operates using URDF (Unified Robot Description Format). An example of this format is provided in 'arm_controller/urdf/ex.urdf'. The library constructs a robot arm out of the URDF and uses it to fill out the PyChain wrapper which is used by the arms as the back end of the arm itself.

### Basic Operation

Basic usage of this library is simple and straightforward:

1. Import the 'arm_controller' library, and any specific arm you would like to use.

   `from arm_controller.arms.mechatronics_arm import MechatronicsArm`

2. Create an arm object.

   `arm = MechatronicsArm()`

3. The arm will auto move to its default position during instantiation of itself.

4. Using the 'set_speed' method you can set the speed (and optionally specify whether it is in rad/s or deg/s).

    `arm.set_speed(10)` # Without specifying, it will assume you entered speed in deg/s.

5. Using the 'get_pos' method you can retrieve the current xyz, rpy of the end effector (end point) of the arm.

    `xyz, rpy = arm.get_pos()`

6. Using the 'move_to' method you specify an x, y, z (in meters) to move the arm. Optionally, you can supply a r, p, y (and optionally specify whether it is in rad/s or deg/s).    This function will return the angles that arm is now set to (in radians).

    `angles = arm.move_to(0.04, 0.06, 0.08, roll=0.78, pitch=0.78, yaw=0.78, radians=True)` # Like 'set_speed' without specifying, it will assume you entered the rpy in degrees.

7. Using the 'set_default_position' method, the arm will auto move from its current position back to its default position.

    `arm.set_default_position()`

8. For the mechatronics arm specifically, you can specify 'open_claw' or 'close_claw' to open or close the claw on the arm respectively. These functions take two optional arguments, one for the angle to open/close the claw to, and the other whether that argument is in radians.

    `arm.open_claw()` # With no arguments supplied, the robot will open its claw to its default open value.
      `arm.close_claw(value=10.0, radians=False)`

9. Finally, the 'set_joint' method takes in a string to designate the joint to move, its new value, and whether the value is in radians or degrees. This method is called by all other methods internally when moving any part of the robot, but can be useful when wishing just to move a single joint at a time or when smaller, more precise, movements may be necessary.

    `arm.set_joint('elbow', 60.0, radians=False)` # to know

**Plotter**: All of this functionality, except for the open/close claw methods, work the same way in the PlotterArm class. The one difference is that you must call the *exit* function before closing the program to properly disconnect the pipes.

**demo.py**: Demos the primary functionality of the robot arm using the plotter class to guarantee functionality as compared to the mechanical arm.

### Advanced Operation

#### Using the Solvers

This library includes several kinematic library wrappers that allow the user to specify which kinematic solver will be used for the robotic arms. The recommended and best working solver at this time is IKPy, however; also included are RTB and Orocos/PyKDL libraries (though these are not as thoroughly tested and may need further development in order to get working properly).

Currently, to switch between solvers, you need to change the import and the line creating the solver in the arm you are using:

    from arm_controller.solvers.ikpy_solver import IKPySolver # remove this line
    from arm_controller.solvers.rtb_solver import RTBSolver # replace it with this line
    ...
    self._solver = IKPySolver(self.chain) # remove this line
    self._solver = RTBSolver(self.chain) # replace it with this line

All solvers have three functions from which to call and can be accessed via the 'arm._solver' variable if you need to use the solver directly without the arm:

1. 'inverse_solve' takes in the 'target_coords' and 'target_rpy' (in radians) as lists and returns the angles the arm needs to be set to in order to reach that point.

2. 'forward_solve' takes in the current angles of the arm and returns the xyz and rpy in two lists of the arms end effector.

3. 'segmented_forward_solve' takes in the current angles of the arm and returns the xyz position of each of the joints as a list of lists.

All solvers arguments and returns of these functions are the same, making the swap between solvers as simple as possible, but the internal implementation will of course change amongst the solvers.

#### Virtualization

In addition to operating a physical arm, users can simulate the arm using a matplotlib animation via the PlotterArm class. This class animates the actions the arm would be taking in physical space.

In order to do this, a separate process is spawned during creation of the class. This process controls the matplotlib animation, as matplotlib needs to be the primary thread of the program in order to run correctly and allows control return back to the user to enter commands.

To safely exit the process you need to call the 'exit()' function which flags the plotter process that it needs to exit.
