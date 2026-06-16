# icai_crl_gazebo
This package contains gazebo simulation configurations, custom plugins and model modifications to complement their description. It is meant to prepare model files from icai_crl_description to be launched in Gazebo robot simulator.

## Usage

### Install

* ROS 2: Humble
* Gazebo: Fortress
* dependencies: icai_crl_description

Clone this repository inside the src folder of a ROS 2 workspace and compile using `colcon build`

### Launch
Worlds can be launched via command `ign gazebo` or via a launch file (for example from package icai_crl_bringup_sim). Models can only be launched if included inside a world or with a launch file.

## Worlds

### empty_world
Contains a ground plane iluminated by a directional light source (the Sun).

### wall_world
Similar to empty_world, but adding a long wall for wall following controllers.

### control_laboratory
ICAI's Control Laboratory model with its lights and furniture. Also includes the Sun to illuminate the street view through the windows.

### control_laboratory_lite
Same laboratory with no lights to reduce computational load.


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


### kitt_md25
Kitt model equipped with the MD25 motor driver plugin that simulates a realistic DC motor control with voltage quantization, current simulation, and encoder feedback.

Transport topics:
* `/model/kitt/imu` (gz.msgs.IMU) IMU with 'x' pointing forward, 'y' pointing right and 'z' pointing down.

* `/model/kitt/front_dist_sensor` and `/model/kitt/back_dist_sensor` (gz.msgs.LaserScan) Lateral sensors to compute distance and orientation relative to a wall.

* `/model/kitt/{joint_name}/motor_volt_cmd` (gz.msgs.Double) Voltage command for each motor.

* `/model/kitt/{joint_name}/motor_output_torque` (gz.msgs.Double) Motor output torque.

* `/model/kitt/{joint_name}/joint_velocity` (gz.msgs.Double) Joint angular velocity.

* `/model/kitt/{joint_name}/motor_voltage` (gz.msgs.Double) Actual motor voltage.

* `/model/kitt/{joint_name}/motor_current` (gz.msgs.Double) Motor current.

* `/model/kitt/{joint_name}/motor_encoder` (gz.msgs.Int32) Encoder count.

### kitt_segway_md25
Similar to kitt_md25, but with lateral sensors reoriented to perform as a self-balancing vehicle in inverted pendulum configuration.

### Model Templates

Each of the models described above (`kitt_dd`, `kitt_nav_dd`, `kitt_md25`) has a corresponding template version located in a directory ending with `_template`.

These templates solve an issue with topic names construction. If a model is spawned in Gazebo with a different name (e.g., for multi-robot simulations), not all topics follow the same naming standard, leading to inconsistencies when using ROS 2 namespaces.

By using the [Jinja](https://jinja.palletsprojects.com/) templating engine, these files allow for the dynamic generation of model configurations. A `model_name` variable is injected into the `.sdf.jinja` and `model.config.jinja` files before they are loaded. This ensures that all transport topics are correctly namespaced with the final name given to the model upon spawning (e.g., `/model/my_robot_name/imu`).

This process is handled automatically by the ROS 2 launch files in the `icai_crl_bringup_sim` package, which allow specifying the desired `model_name` as a launch argument.


## Plugins

### MD25 Plugin

The MD25 plugin simulates the behavior of an MD25 dual motor driver board, providing realistic DC motor control with voltage quantization, current simulation, and encoder feedback.

#### Parameters:

* `left_joint`: Name of the left motor joint (required)
* `right_joint`: Name of the right motor joint (required)
* `electromotive_force_constant`: EMF constant in Nm/A (default: 0.539111)
* `electric_resistance`: Motor resistance in Ohms (default: 7.101)
* `electric_inductance`: Motor inductance in Henry (default: 0.0034)
* `diff_voltage_drop`: Differential voltage drop between motors (default: -0.01048)
* `gear_ratio`: Gear ratio motor to output (default: 1.0)
* `encoder_ppr`: Encoder pulses per revolution (default: 360)
* `encoder_rate`: Encoder publishing rate in Hz (default: 200)
* `max_update_steps`: Maximum register update steps (default: 10)
* `performance_mode`: Enable performance mode (default: true)
* `voltage_update_period`: Voltage update period in ms (default: 25)
* `left_volt_cmd_topic`: Custom topic for left motor voltage commands (optional)
* `right_volt_cmd_topic`: Custom topic for right motor voltage commands (optional)

#### Topics:

* `/model/{model_name}/{joint_name}/motor_volt_cmd` (msgs::Double): Voltage command
* `/model/{model_name}/{joint_name}/motor_output_torque` (msgs::Double): Motor output torque
* `/model/{model_name}/{joint_name}/joint_velocity` (msgs::Double): Joint angular velocity
* `/model/{model_name}/{joint_name}/motor_voltage` (msgs::Double): Actual motor voltage
* `/model/{model_name}/{joint_name}/motor_current` (msgs::Double): Motor current
* `/model/{model_name}/{joint_name}/motor_encoder` (msgs::Int32): Encoder count

