# Robotic Arm Control Using Forward & Inverse Dynamics with KUKA iiwa14

## Project Overview
This project implements dynamic control for the KUKA iiwa14 robotic arm using forward and inverse dynamics. The main goal is to simulate and control the robot's movement through trajectory planning, computation of joint torques, and joint accelerations using ROS and Python.

## Features
- **Forward & Inverse Kinematics**: Implemented using DH parameters and the KDL library.
- **Dynamic Models**: Computed dynamic models for joint torques and accelerations using KDL chain solvers.
- **Payload Estimation**: Estimated payload mass and center of mass with less than 5% error, validated through dynamic simulations.

## Files
- `iiwa14DynStudent.py`: The main file for computing forward and inverse dynamics.
- `force_torque_simulation_kuka.py`: A script to load and publish trajectories, compute joint accelerations, and plot results from a bagfile.
- `force_torque_simulation_kuka.bag`: Bag file containing trajectory data.
- `iiwa14.rviz`: RViz configuration file for visualizing the robot's motion.
- `setup.py`: Script for packaging the ROS workspace.

## Usage
1. Install the required ROS packages by running `setup.py` using catkin.
2. Launch the simulation environment using `force_torque_simulation_kuka.launch`.
3. Execute the main script using the command:
   ```
   rosrun force_torque_simulation_kuka force_torque_simulation_kuka.py
   ```
4. Use RViz to visualize the trajectory and joint accelerations.

## Dependencies
- ROS Melodic/Noetic
- KDL Library for kinematics
- Matplotlib for plotting joint accelerations
- Python 3.x

## License
This project is licensed under the MIT License.