# aero\_ros\_controller

## Provided Nodes

### aero\_ros\_controller
- This node provides joint trajectory controller and low level base controller
- Controllers using [ros\_control] ( http://wiki.ros.org/ros\_control )

- Published topics
  - odom \[nav\_msgs/Odometry\]
    - odometry

  - tf \[tf2\_msgs/TFMessage\]
    - represening odometry by tf
    
  - joint\_states \[sensor\_msgs/JointState\]
    - using [joint\_state\_controller] ( http://wiki.ros.org/joint\_state\_controller ) 
    
- Subscribed topics
  - cmd\_vel
    - low level velocity command for wheels

  - <name>\_controller
    - providing follow\_joint\_trajectory \[action\] 
    
- Parameters
  - controller\_rate
    - rate of read/write cycle \[ Hz \]
    
  - overlap\_scale
    - scaling of target duration for each command cycle

### aero\_hand\_controller
- This node provides device independent hand control servie

- Subscribed topics
  - joint\_states
    - actual angles for controllers

  - lhand\_controller
    - joint control for left hand 
    - follow\_joint\_trajectory [action]
    
  - rhand\_controller
    - joint control for right hand
    - follow\_joint\_trajectory [action]
    
- Services
  - /aero\_hand\_controller [aero\_startup/HandControl]
