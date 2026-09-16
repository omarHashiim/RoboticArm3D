# 3D Robotic Arm Simulator — MATLAB

A 3D interactive robotic arm simulation built with MATLAB.

## Project Overview

This project simulates a 4-DOF robotic arm with an interactive graphical interface.

The simulator allows users to control the robot joints, adjust the gripper, visualize the robot in 3D, and observe the end-effector movement.

## Features

* 3D robotic arm visualization
* 4-DOF joint control
* Interactive GUI
* Forward Kinematics
* End-effector XYZ position tracking
* Adjustable gripper
* Robot animation
* End-effector trajectory visualization
* Reset functionality
* 3D coordinate system
* Cylindrical robotic links and joints

## Technologies

* MATLAB
* MATLAB GUI
* 3D Visualization
* Forward Kinematics
* Robotics Concepts

## Robot Structure

The robotic arm consists of:

* Base Platform
* Vertical Column
* Shoulder Joint
* Elbow Joint
* Wrist Joint
* End Effector
* Gripper

## How to Run

1. Open MATLAB.
2. Open the project folder.
3. Open `robotic_arm_gui.m`.
4. Run the following command in the MATLAB Command Window:

```matlab
robotic_arm_gui
```

5. Use the sliders to control the robot joints.
6. Adjust the gripper using the Gripper slider.
7. Press `ANIMATE` to run the predefined robot motion.
8. Press `RESET` to return the robot to its initial position.

## Forward Kinematics

The simulator calculates the position of the end effector using the robot joint angles and link lengths.

The resulting position is displayed in real time as:

```text
X = ...
Y = ...
Z = ...
```

## Project Structure

```text
RoboticArm3D/
│
├── robotic_arm_gui.m
├── README.md
│
└── screenshots/
```

## Future Improvements

Possible future extensions include:

* Inverse Kinematics
* Pick-and-place simulation
* Object detection
* Collision detection
* Path planning
* Real robotic arm integration
* More advanced DH parameter modeling

## Author

Omar Hashim

MATLAB Robotics & 3D Simulation Project
