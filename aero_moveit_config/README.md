# aero\_moveit\_config

### udev setting
```
sudo cp ~/{catkin_workspace}/src/aero-ros-pkg/aero_startup/aero_hardware_interface/udev/90-aero.rules /etc/udev/rules.d/

Please edit 90-aero.rules as below, and reconnect usb interfaces. 

# aero controller
SUBSYSTEMS=="usb",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A907BH3E",MODE="666",SYMLINK+="aero_upper"
SUBSYSTEMS=="usb",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="AE017404",MODE="666",SYMLINK+="aero_lower
```

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
