# aero_description

- author: Kazuhiro Sasabuchi

The package manages robot hardware configuration.

## Basic Usage

To set up robot configuration, just run `setup.sh`.

```
./setup.sh [CONFIGDIR]
```

Or if you do not want to build robot controllers but need to create robot models, add `--local` option.

```
./setup.sh [CONFIGDIR] --local
```

If you want to clean up, use `clean.sh`.

```
./clean.sh
```

### Break Down

`setup.sh` will call shell scripts in `scripts` directory.

- `setup_models_directory.sh`
    - copy euslisp models into `aero_description/models`
- `create_urdf.sh`
    - copy `urdf`, `robots`, and `meshes` into `aero_description`
- `install_srv.sh`
    - create `aero_startup/CMakeLists` and add service files

If `--local` is added, previous 3 scripts will run,
else followings will run after these.

- `make_controllers.sh`
    - generate controllers into `aero_startup/aero_hardware_interface`
- `make_joint_state_publisher.sh`
    - generate `aero_startup/aero_controller_manager/AeroJointStatePublisher.cc`
- `make_angle_to_stroke_header.sh`
    - generate `aero_startup/aero_hardware_interface/Angle2Stroke.cc` using csv files under `models/csv`
- `make_stroke_to_angle_header.sh`
    - generate `aero_startup/aero_hardware_interface/Stroke2Angle.cc` using csv files under `models/csv`
- `unused_angle_to_stroke.sh`
    - generate `aero_startup/aero_hardware_interface/UnusedAngle2Stroke.hh`
- `configure_controllers.sh`
    - insert additional controllers into `aero_startup/CMakeLists.txt` and generate `aero_startup/generated_controllers.launch`

## Detail

### Controller

The robot structure is assumed to have 2 SEED controllers,
upper and lower body.

`headers/Constants.hh` is a configuration file including;

- CAN ID of each SEED controller
- DOF of each upper / lower part
- Indices of each actuators in stroke and raw commands


### Angle <-> Stroke

The robot has some LINK joints, using prismatic actuators.
These prismatic actuators are operated by stroke length,
however typical robot software frameworks (OpenHRP and ROS)
using only rotational joints and rotation angles as q vector.
To apply to these frameworks,
the robot has conversion table
between joint angle and stroke length.

#### AngleToStroke

#### StrokeToAngle

#### UnusedAngleToStroke

## Create New Configuration

### Directory Structure

The configuration directory MUST contain followings.

- controllers
    - (controller sources)
- headers
    - (controller headers)
- models
    - csv
        - {joint_name}.csv
- aero_upper.txt
- aero{$LOWER}.txt
- controllers.cfg

### Syntax of controllers.cfg

- `&` : MERGE into controller class
- `+` : ADD standalone node into CMakeLists.txt and launch file
