# aero_move_base

## AeroBaseControllerNode

This node is for Vehicle-type lower body,
and be automatically added generated_controllers.launch.
`aero_bringup.launch` will run this.

To use this, put `AeroBaseController.cc` on robot's description directoly
and define `AeroMoveBase::Init` and `AeroMoveBase::VelocityToWheel`.

Following files are sources of this Node;

- Main.cc
- AeroBaseController.cc (Auto Generated)
- AeroMoveBase.{cc,hh}

## Joystick Teleoperation

```
roslaunch wheel_bringup.launch
```


## Navigation

### Mapping

```
roslaunch wheel_with_making_map.launch
```

will launch `wheel_bringup` and mapping nodes.

For running mapping nodes on external PCs,
launch `wheel_bringup.launch` on the Robot control PC first,
then

```
roslaunch making_map_navigation.launch
```


### Navigation

```
roslaunch wheel_with_static_map.launch
```

will launch `wheel_bringup` and navigation nodes.

For running mapping nodes on external PCs,
launch `wheel_bringup.launch` on the Robot control PC first,
then

```
roslaunch static_map_navigation.launch
```


### For more information,

please refer the Documentation of the ROS navigation stack.

http://wiki.ros.org/navigation
