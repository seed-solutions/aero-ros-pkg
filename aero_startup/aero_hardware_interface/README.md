# aero_hardware_interface

This directory contains hardware controller interface.

## AUTO GENERATED FILES: DO NOT EDIT

This package (after running `aero_description/setup.sh`)
contains AUTO GENERATED FILES.
DO NOT EDIT these files, if you edit these files directory,
you can not commit any changes into repository,
also you can not check diff using `git diff`.

Followings are AUTO GENERATED FILES.
If you want to edit, please edit original files in `.templates` and
robot configuration directory under `aero_description`.

- `AeroControllers.cc`
- `Constants.hh`
- `Angle2Stroke.*`
- `Stroke2Angle.*`
- `AngleJointNmaes.*`
- `UnusedAngle2Stroke.hh`

## Interfaces

### AeroControllerProto

AeroControllerProto.{hh,cc} contains following 2 classes;

- SEED485Controller
- AeroControllerProto

SEED485Controller is a communication interface
via USB/RS485 from PC to SEED Micom.
This contains simple I/O method using SEED protocol.

AeroControllerProto is a wrapper class
including commands to control actuators and
to read status of each smart actuators.
AeroControllerProto is not a subclass of SEED485Controller
but it has an instance of SEED485Controller.

### AeroControllers (AUTO GENERATED)

AeroControllerProto has only commands to control raw rotation of actuators,
and has no information about joint type or angle.
AeroControllers.{hh,cc} has AeroUpperController and AeroLowerController,
these are subclass of AeroControllerProto,
joint information and some special behaviors (like wheels) are defined
in these classes.
