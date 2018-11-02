# aero\_moveit\_config

### How to use moveit without real robot

```
roslaunch aero_moveit_config demo.launch
```

### How to use moveit with real robot
1. If USB connect your local PC from robot_pc, 

udev_setting

First, 
```
sudo cp ~/{your_workspace}/src/aero-ros-pkg/aero_startup/aero_hardware_interface/udev/90-aero.rules /etc/udev/rules.d
```

Second, please connect only the USB connector from upper_body,
```
udevadm info /dev/ttyUSB0 | grep "ID_SERIAL_SHORT" 
```
Please overwrite the displayed serial number in /etc/udev/rules.d/90-aero.rules

For example, 
```
ATTRS {idProduct} == "6001", ATTRS {serial} == "FT98HN7S", MODE = "0403", ATTRS {idVendor} == "0403", ETR: ID_SERIAL_SHORT = FT98HN7S, 666 ", SYMLINK + =" aero_upper "
```
Similarly, plug and unplug USB in lower body (command confirmed)


and 
```
roslaunch aero_startup aero_bringup.launch
roslaunch aero_moveit_config move_group.launch
roslaunch aero_moveit_config moveit_rviz.launch config:=1
```


