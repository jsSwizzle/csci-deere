# Robot Arm Library

This library was developed by NDSU CSCI 445 students in the capstone class in the Spring 2021 semester for John Deere Electronics Solutions.

## Installation

This library requires at least Python Version 3, and is OS Independent without the use of the Orocos KDL library.

1. Begin by cloning this repository inside your workspace
2. *cd* into the directory
3. Depending on your needs execute one of the following commands
    * To simply install to Python run: `pip3 install .`
    * To install in developer mode run: `pip3 install -e .` (developer mode sets a link to this directory, and allows for a live working copy to be used)

**If you are using Orocos KDL Library:** follow the instructions here `github.com/orocos/orocos_kinematics_dynamics` to download and install the Orocos Kinematics and Dynamics Library (**Note:** this installation assumes you using a Debian/Ubuntu OS!)

## Features

* Moves robotic arm to XYZ position in cartesian space
* Enables arm to open and close hand around object
* Enables arm to rotate hand for proper orientation of object
* Calculates forward and reverse kinematics of arm

## Usage

*Insert usage instructions for operating library*
