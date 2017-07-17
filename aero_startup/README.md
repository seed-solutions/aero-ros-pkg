# aero_startup

## NOTICE

This directory is NOT a ROS PACKAGE before running `aero_description/setup.sh`.
Please refer to `aero_description` at first.

## AUTO GENERATED FILES: DO NOT EDIT

This package (after running `aero_description/setup.sh`)
contains AUTO GENERATED FILES.
DO NOT EDIT these files, if you edit these files directory,
you can not commit any changes into repository,
also you can not check diff using `git diff`.

Followings are AUTO GENERATED FILES.
If you want to edit, please edit original files in `.templates` and
robot configuration directory under `aero_description`.

- `CMakeLists.txt`
- `generated_controllers.launch`
- `aero_controller_manager/`
  - `AeroJointStatePublisher.cc`
  - `AeroTorsoController.cc`
  - `AeroHandController.cc`
- `aero_hardware_interface/`
  - `AeroControllers.cc`
  - `Constants.hh`
  - `Angle2Stroke.*`
  - `Stroke2Angle.*`
  - `AngleJointNmaes.*`
  - `UnusedAngle2Stroke.hh`
- `aero_move_base/`
  - `AeroBaseController.cc`

## BASIC USAGE

```
roslaunch aero_startup aero_bringup.launch
```
