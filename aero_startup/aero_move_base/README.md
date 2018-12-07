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

## Move on Rviz without real robot

```
roslaunch aero_startup wheel_with_dummy.launch
```


## Navigation

### Mapping

```
roslaunch aero_startup wheel_with_making_map.launch
```
After making map, you should run ```roslaunch aero_startup map_saver.launch``` to save map.


### Navigation

```
roslaunch aero_startup wheel_with_static_map.launch
```

### Local Planner Setting
You can choose base local planner from `TrajectoryPlannerROS`, `DWAPlannerROS`, `EBandPlannerROS`, `TebLocalPlannerROS`. (`TrajectoryPlannerROS` and `DWAPlannerROS` are not supported, now.)
Default planner is `TebLocalPlannerROS`, described in move_base.launch.

Features are as follows:
* [teb_local_planner/TebLocalPlannerROS](http://wiki.ros.org/teb_local_planner)  
to avoid collision with people and when moving long distances, default settings do not allow moving sideways (this avoids the robot from moving diagonally in a straight path)

* [eband_local_planner/EEBandPlannerROS](http://wiki.ros.org/eband_local_planner)  
to move sideways, if your robot is moving in a small area or is using the base to make small adjustments during manipulation

### For more information,

please refer the Documentation of the ROS navigation stack.

http://wiki.ros.org/navigation
