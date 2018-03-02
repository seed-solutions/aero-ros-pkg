# aero_description

- author: Kazuhiro Sasabuchi

The package manages robot hardware configuration.
One of SEEDNOID's feature is easily change and customize its component such as hand or lower body.
This package provides setup procedure from user hardware configuration.

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
- `create_eusmodel.sh`
  - [Optional] generate eusmodel `aero.l` if exist `robots/aero.{urdf.xacro,yaml}`.
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

### Configuration

User can choose hardware component from `aero_shop`.
`typeF` is the default configuration.
This contains following files.

- `aero_upper.txt`
- `aero-wheels.txt`
- `robot.cfg`

`robot.cfg` contains parts construction.

### Controller

The robot structure is assumed to have 2 SEED controllers,
upper and lower body.
`AeroControllerNode` has `AeroUpperController` and `AeroLowerController`,
both controllers are inherited from `AEROControllerProto` and `SEED485Controller`.

`headers/Constants.hh` is a configuration file including;

- CAN ID of each SEED controller
- DOF of each upper / lower part
- Indices of each actuators in stroke and raw commands

When running `make_contrllers.sh`,
joint information will be written into `AngleJointNames.hh`.

Also `configure_controller.sh` will generate additional controllers,
which are defined in `robot.cfg`.
If parts dir contains `controller` dir, scripts will copy controller from it.
`->` operator means to set target directory under `aero_startup`,
ex) mobile base controller will be copied into `aero_move_base` under `aero_startup`

### Angle <-> Stroke

The robot has some LINK joints, using prismatic actuators.
These prismatic actuators are operated by stroke length,
however typical robot software frameworks (OpenHRP and ROS)
using only rotational joints and rotation angles as q vector.
To apply to these frameworks,
the robot has conversion table
between joint angle and stroke length.

Conversion table is provided as csv files.
A csv file contains following information for each degrees.

```
degree, CAD measure[mm], relative stroke[mm], absolute stroke[mm], relative pulse, absolute pulse
```

Note that only degree, relative stroke, absolute stroke is used for generating the header files.


#### AngleToStroke

convert joint angle to stroke length

#### StrokeToAngle

convert stroke length to joint angle

#### UnusedAngleToStroke

UnusedAngleToStroke creates a masking function.
Angles not used has to be defined as 0xffff in stroke.
UnusedAngleToStroke handles this mapping.


## [Advanced] Create New Configuration

** NOTICE ** :
This is an optional step for creating a new robot configuration directory.
This step is not required if you are using an existing robot configuration directory.

### Directory Structure

The configuration directory MUST contain followings.

- headers
  - Angle2Stroke.hh
  - Stroke2Angle.hh
  - Constants.hh
- moveit_config
  - config
    - AeroUpperRobot.srdf
    - controllers.yaml
    - fake_controllers.yaml
    - joint_limits.yaml
    - kinematics.yaml
    - ompl_planning.yaml
- robots
  - aero.urdf.xacro
  - aero_moveit_ik.urdf.xacro
  - aero_moveit_ik_op.urdf.xacro
  - aero_moveit_ik_ho.urdf.xacro
- urdf
  - aero-upper.urdf.xacro
  - aero-{$LOWER}.urdf.xacro
  - common.xacro
- aero_upper.txt
- aero-{$LOWER}.txt
- robot.cfg
