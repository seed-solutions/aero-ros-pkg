# aero\_moveit\_config


### How to control SEED-Noid
```
connect wifi:
SSID:SEED-A-3ED0
pass:n4u3gewyky5a3

export $ROS_MASTER_URI = http://192.168.0.29:11311

roslaunch aero_moveit_config move_group.launch
roslaunch aero_moveit_config moveit_rviz.launch config:=1

```

### How to use moveit without real robot

```
roslaunch aero_moveit_config demo.launch
```
