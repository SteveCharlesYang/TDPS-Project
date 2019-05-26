# TDPS Presentation

## Whole Structure

The car has a three-layer structure, each with different components.
In the position of the components, the position of the center of gravity and the influence on the speed are taken into consideration.

## ROS

ROS is a computer operating system architecture designed for robot software development. It is an open source, meta-level operating system that provides services similar to operating systems, including underlying driver management, shared function execution and inter-program messaging.

The control system has two topological links, each line is an abstract relationship of program nodes inside the system.

### Main line

The main line is used to accomplish the main goal of the mission. It starts with the Missions section and ends with Motors, with a built-in auto navigation algorithm.

Missions -> Goals -> AMCL -> Planner -> move_base -> cmd_vel/odom -> base_controller -> Motor

#### Missions: Task Scheduler

Controls the launch of the corresponding high-level ROS launch file.

#### Goals: Target Scheduler

Each Goals is a different task node within each ratio, and the same type of task is the same under the same goal. 

#### AMCL(Adaptive Monte Carlo Localization): Positioning System

AMCL is a probabilistic positioning system for mobile robots in a two-dimensional environment.

#### Planner: GlobalPlanner and LocalPlanner

GlobalPlanner uses the Dijkstra optimal path algorithm to plan the route of the whole process.

LocalPlanner plans the segment distance route in the next few seconds with DWA(Dynamic Window Approaches).

#### move_base: Navigation library

Move_base converts navigation plans into speed and direction instructions.

#### cmd_vel: Motion Control Message

Cmd_vel passes the control commands directly through the base_controller.

#### base_controller: Basic Control Node

It contains the basic PID algorithm to control motor motion.

#### motor: Car Chassis

The car chassis, driven by Arduino, is only used as a pure hardware controller, and all the logic is placed on the Raspberry Pi running ROS.

### Navigation and Calibration line

The navigation and calibration line is a closed loop control line used to correct the odometer's error and determine the position.

Encoder / 9-axis sensor -> odom / imu -> odom_combined -> map

#### odom: odometer

The odometry message is the mileage of the car, which refers to the distance of the motion obtained by the sensor.

#### imu: inertial measurement unit

It is a device that measures the three-axis attitude angle and acceleration of an object.

#### ekf: Extended Kalman Filter

Used to evaluate the pose of the car, using pose measurement information from different sources.
