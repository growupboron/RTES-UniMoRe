# RTES-UniMoRe

This is a course project for the postgraduate level course of [Real Time Embedded Systems](https://unimore.coursecatalogue.cineca.it/insegnamenti/2022/24604/2021/10003/10300?coorte=2022&schemaid=20417) taught at [DIEF, UniMoRe](https://inginf.unimore.it/laurea-magistrale-ing-inf/).

Project objectives mentioned [here](./Objectives/).

## Project 1: PID Control of the Pole Cart System

0. Setup ROS 2: https://docs.ros.org/en/foxy/Installation.html
1. Setup Cart Pole simulation gazebo environment: https://github.com/cscribano/gazebo_polecart_ros
2. This repository is structured as a ROS2 workspace; clone it:
   ```
   $ git clone git@github.com:growupboron/RTES-UniMoRe.git
   ```
3. Build the cart_pole_controller package:
   ```
   $ cd RTES-UniMoRe/polecart_ws
   $ colcon build --symlink-install
   ```
   
6. _(If required)_ Tune the PID controller:
   ```
   $ vim polecart_ws/src/cart_pole_controller/config/cart_pole_params.yaml
   ```
7. Launch the simulation:
   ```
   $ source /opt/ros/$ROS_DISTRO/setup.bash # ROS 2 Environement
   $ source /<path-to>/gazebo_polecart_ros/install/setup.bash # Pole Cart Simulation Environment
   $ ros2 launch gazebo_polecart_ros polecart.launch.py
   ```
8. _(In a separate terminal instance)_ Launch the controller:
   ```
   $ source /opt/ros/$ROS_DISTRO/setup.bash # ROS 2 Environment
   $ source /<path-to>/RTES-UniMoRe/polecart_ws/install/setup.bash # Pole Cart Controller
   $ ros2 launch cart_pole_controller cart_pole_controller_launch.py
   ```
_(Repeat environment sourcing for each new terminal or put it into the .bashrc file.)_

### PID Explanation

#### Proportional Gain (kp):
- Effect: Determines the reaction to the current error. A higher kp results in a stronger correction.
- Too High: Can cause oscillations or an explosive response.
- Too Low: The system will respond sluggishly and may not correct the error effectively.

#### Integral Gain (ki):
- Effect: Addresses accumulated past errors. It helps eliminate the steady-state error.
- Too High: Can lead to slow oscillations and instability.
- Too Low: The system may have a steady-state error.

#### Derivative Gain (kd):
- Effect: Reacts to the rate of change of the error. It helps dampen the oscillations.
- Too High: Can cause the system to react too quickly to changes, leading to instability.
- Too Low: The system may not dampen oscillations effectively.

### PID Tuning Procedure:

1. Set ki and kd to Zero: Start by setting ki and kd to zero. Increase kp until the system oscillates around the setpoint.

2. Increase kd: Gradually increase kd to dampen the oscillations. This helps stabilize the system.

3. Adjust ki: Finally, increase ki to eliminate any steady-state error. Be careful, as too high a ki can reintroduce instability.
