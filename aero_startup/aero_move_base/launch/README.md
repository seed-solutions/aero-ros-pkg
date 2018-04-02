## combined launch

### wheel_with_static_map.launch
- files
  - ```wheel_bringup.launch```
  - ```static_map_navigation.launch```

### wheel_with_making_map.launch
- files
  - ```wheel_bringup.launch```
  - ```making_map_navigation.launch```

## launching few nodes

### wheel_bringup.launch
- nodes
  - ```urg_node```
  - ```joy_node```
  - ```teleop_twist_joy```

### static_map_navigation.launch
- nodes
  - ```map_server```
- files
  - ```amcl.launch```
  - ```move_base.launch```

### making_map_navigation.launch
- files
  - ```move_base.launch```
  - ```gmapping.launch```

## setting parameters

### amcl.launch
amcl settings (localization using map and laser scan)

- nodes
  - ```/amcl```

### move_base.launch
move_base settings (path planning)

- nodes
  - ```/move_base```

### gmapping.launch
gmapping settings (slam for making map)

- nodes
  - ```/slam_gmapping```

### map_saver.launch
saving map

- nodes
  - ```/map_saver```
