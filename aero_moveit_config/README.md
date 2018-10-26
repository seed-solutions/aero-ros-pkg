# aero\_moveit\_config

### If you want to control SEED-Noid, please login robot PC way to SSH,
```
connect wifi:
SSID:SEED-A-3ED0
pass:n4u3gewyky5a3

In each terminal,
export ROS_MASTER_URI=http://192.168.0.29:11311 
export ROS_IP=`hostname -I` 
export ROS_HOSTNAME=`hostname -I` 

and 
roslaunch aero_moveit_config move_group.launch
roslaunch aero_moveit_config moveit_rviz.launch config:=1

```

### How to use moveit without real robot

```
roslaunch aero_moveit_config demo.launch
```

### How to use moveit with real robot

```
roslaunch aero_moveit_config move_group.launch
roslaunch aero_moveit_config moveit_rviz.launch config:=1
```

