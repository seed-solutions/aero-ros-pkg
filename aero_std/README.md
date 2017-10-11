# aero_std

- Author: Shintaro Hori

## Description

This package provides standard interfaces to use AERO

## Library

### aero_moveit_interface

This requires spot_manager and its service '/get_spot'

### object_features

This includes some functions to use `visualization_msgs/Marker`.


## Node

### spot_manager

spot_manager is a manager of spot in a map.
All spots are saved as yaml into `aero_std/spot.yaml` by the default.


## srv

### GetSpot

Get designated spot as geometry_msgs/Pose

#### request

- string name

#### response

- bool status
- string error
- geometry_msgs/Pose pose

### GetSpots

Get all spot name

#### request

- None

#### response

- string[] spots

### SaveSpot

Save current position as spot with designated name

#### request

- string name

#### response

- bool status

### DeleteSpot

Delete designated spot

#### request

- string name

#### response

- bool status
