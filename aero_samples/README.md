# aero_samples

- Author: Shintaro Hori

## Description

This package provides sample code to use `aero_std`

## Sample nodes

### minimum_sample_node

This is the minimum sample to use `aero_moveit_interface`,
sends only `ResetManipPose`.

### fk_sample_node

Forward Kinematics (FK) sample.

### ik_sample_node

Inverse Kinematics (IK) sample.

### interpolation_sample_node

`aero_moveit_interface` has some different interpolation mode.
This sample shows how to switch interpolation mode to Linear and Sigmoid.

### hand_sample_node

Hand control sample.

### look_at_sample_node

"Look at" means to move head angle forwarding designated target e.g. robot hand.

### look_at_external_sample_node

This is target tracking sample using static target or external topic (`geometry_msgs/Pose`).

### top_grasp_sample_node

Top grasp sample using `aero::TopGrasp`.

### side_grasp_sample_node

Side grasp sample using `aero::SideGrasp`.

### trajectory_lifter_sample_node

This sample is to set lifter trajectory.

### go_pos_sample_node

Wheel mover sample.
This requires to launch `aero_startup/wheel_with_making_map.launch`.

### async_sample_node

Asynchronous control sample.

### async_lifter_cancel_node

Asynchronous lifter control (send goal / cancel) sample.

### set_marker_sample_node

This is sample to use `aero::vision::ObjectFeatures`.

## Test nodes

All tests recommend to use real robot.

### stoa_atos_test_node

Test for interconversion of `Stroke2Angle` and `Angle2Stroke`.

### fullbody_test_node

Full body joint control test.

### lifter_ik_test_node

Lifter control test.

### upper_controller_test_node

Upper body control test.
