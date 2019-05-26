# TDPS Presentation

## Line-following Algorithm

### Public section

- First grayscale the image and reduce the size to the processing power of the Raspberry Pi.
- Call the edge detection process to extract the "roughness" feature of the road image.
- Then use the open operation to increase the contrast of the image.
- Divide the image into two parts, and do the same for each of the two images.
- For each image matrix after segmentation, a summation operation is performed on the pixels of each column to obtain a distribution in the horizontal direction of the feature describing the "roughness".
- Subtract the average of the series and multiply it by the unit step function.
- The distribution array obtained in the previous step is weighted by index and divided by the average of the series to obtain a coordinate that reflects the direction of the road surface on this part of the image.

### Initial part

- Repeat the above process to obtain the distribution function and its parameters that best fit the initial state, and then use the parameters to go to the task section.

### Task part

- Make the difference between the upper and lower parts and judge whether it is within the error range of the neutral line, otherwise the steering command will be issued according to the situation.
- The steering command is measured by the degree of deviation and is divided into two parts: steering and correcting the lateral offset. The former is to send the steering command, the latter is to send the steering command first, and then send the steering command in the opposite direction and size.

## ROS

ROS is a computer operating system architecture designed for robot software development. It is an open source, meta-level operating system that provides services similar to operating systems, including hardware abstraction descriptions, underlying driver management, shared function execution, inter-program messaging, and program distribution package management. It also provides tools and libraries. A program for acquiring, building, writing, and executing multi-machine fusion.

### Main line

Missions -> Goals -> AMCL -> Planner -> move_base -> cmd_vel/odom -> base_controller -> Motor

### odom spur line

Encoder / 9-axis sensor -> odom / imu -> odom_combined -> map

### Missions: Task Scheduler

Controls the launch of the corresponding high-level ROS launch file. The pre-defined options are used to switch between ratio1 and ratio2, and the task list is sent to the lower-level node to monitor each goal of the task. It is the first part of the system that starts and ends last.

### Goals: Target Scheduler

Each Goals is a different task node within each ratio, and the same type of task is the same under the same goal. It is essentially a Service in ROS, which is started by the task scheduler of the upper level, and will feed back the results and accept the request to execute the action.

### AMCL: Positioning System

AMCL (Adaptive Monte Carlo Localization) is a probabilistic positioning system for mobile robots in a two-dimensional environment. It implements an adaptive Monte Carlo positioning method in which a particle filter is used to track the pose of a robot for an existing map.

### Planner: Planner

For the coordinate system established based on the sensor information, and the map and route given by the task, GlobalPlanner uses the Dijkstra optimal path algorithm to plan the route of the whole process and calculate the minimum cost path on the costmap as the global route of the robot. The car can automatically avoid obstacles during the global planning process without affecting the global path. Since the sensor can dynamically update the costmap, the obstacle can be dynamic (such as a person walking past).

Use LocalPlanner to plan the segment distance route in the next few seconds. The Dynamic Window Approaches have better effects in local obstacle avoidance. The process of the algorithm is:

- The current state of the sampling robot (dx, dy, dtheta)
- For each sampled speed, calculate the state of the robot after traveling at that speed for a period of time to get a route to travel.
- Score multiple routes using some evaluation criteria.
- Based on the score, select the optimal path and repeat the above process.

### move_base: Navigation library

Move_base provides the configuration, operation, and interaction interface of ROS navigation, through which the plan for running the program is finally converted into speed and direction instructions.

### cmd_vel: Motion Control

Cmd_vel is a type of Twist message, which is responsible for passing the control commands of each program to the chassis directly through the base_controller. Sub-messages have linear and angular to reflect angular velocity and line velocity, giving the robot the only motion state.

### odom: odometer

The odometry message is the mileage of the car, which refers to the distance of the motion obtained by the sensor, and the actual mileage information is obtained by the encoder data combined with the inertial navigation by the extended Kalman filter.

### imu: inertial measurement unit

It is a device that measures the three-axis attitude angle and acceleration of an object. To measure the angular velocity and acceleration of an object in three-dimensional space, and to calculate the attitude of the object. As the ATRAS (Attitude and Heading Reference System), it provides inertial data for the Raspberry Pi with ROS, and corrects the accumulation error of odom by ekf.

### ekf: Extended Kalman Filter

Used to evaluate the pose of the car, using pose measurement information from different sources, using an extended Kalman filter with 6-DOF model information to integrate data from the wheel odometer, IMU sensor and visual odometer . The basic idea is to use loose coupling to fuse different sensor information to achieve pose estimation.

### base_controller: Basic Control Node

It is the basic control node in ROS, which contains the basic PID algorithm to control motor motion.

### motor:car chassis

The car chassis, driven by Arduino, is only used as a pure hardware controller, and all the logic is placed on the Raspberry Pi running ROS, and the Arduino program is controlled by serial commands.

At the same time, this chassis control drive can also read the value of the sensor and control the operation of the servo.