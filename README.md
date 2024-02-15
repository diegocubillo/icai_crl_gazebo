# icai_crl_gazebo
This package contains gazebo simulation configurations, custom plugins and model modifications to complement their description. It is meant to prepare model files from icai_crl_description to be launched in Gazebo robot simulator.

## Usage

### Install

* ROS 2: Humble
* Gazebo: Fortress
* dependencies: icai_crl_description

Clone this repository inside the src folder of a ROS 2 workspace and compile using `colcon build`

### Launch
Worlds can be launched via command `ign gazebo` or via a launch file (for example from package icai_crl_bringup). Models can only be launched if included inside a world or with a launch file.

## Worlds

### empty_world
Contains a ground plane iluminated by a directional light source (the Sun). 

### control_laboratory
ICAI's Control Laboratory model with its lights and furniture. Also includes the Sun to illuminate the street view through the windows.


## Models

### kitt_dd
Kitt model equiped with Gazebo's `diff_drive` plugin.
Transport topics:
* `/model/kitt/imu` (gz.msgs.IMU) IMU with 'x' pointing forward, 'y' pointing right and 'z' pointing down.

* `/model/kitt/front_dist_sensor` and `/model/kitt/back_dist_sensor` (gz.msgs.LaserScan) Lateral sensors to compute distance and orientation relative to a wall.

* `/model/kitt/cmd_vel` (gz.msgs.Twist) Velocity commands for the diff_drive plugin.

* `/model/kitt/odometry` (gz.msgs.Odometry) Odometry published by diff_drive plugin.

* `/model/kitt/tf` (gz.msgs.Pose_V) Tf frame of the robot, published by diff_drive plugin.


### kitt_nav_dd
Extension of kitt sensorisation for navigation purposes. It includes the following topic:
* `/model/kitt/rplidar_a2m8` (gz.msgs.LaserScan) RPLIDAR A2M8 360º LiDAR configured with its default parameters.


### kitt_fpv_dd

Kitt model with two cameras to generate either a FPV or a VR front view. These cameras are weightless and fixed to the original model.
* `/model/kitt/left_fpv_camera` and `/model/kitt/right_fpv_camera` (gz.msgs.Image) Images from each eye's camera (320x240).


## Plugins

TODO Include MD25 plugin, add models using it and write documentation. Don't forget to update dependencies.

