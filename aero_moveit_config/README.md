# aero\_moveit\_config

### How to use moveit without real robot

```
roslaunch aero_moveit_config demo.launch
```

### How to use moveit with real robot
```
roslaunch aero_startup aero_bringup.launch
roslaunch aero_moveit_config move_group.launch
roslaunch aero_moveit_config moveit_rviz.launch config:=1
```


