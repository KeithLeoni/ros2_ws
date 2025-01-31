# Robotics Project

[![Watch the video](https://img.youtube.com/vi/5d4sLjeiZd0/sddefault.jpg)](https://youtu.be/5d4sLjeiZd0)



## Other videos
[Watch the video](https://www.youtube.com/watch?v=AHXESDaahys&ab_channel=LucaHardonk)

[Watch the video](https://youtu.be/Xyv_HV6b0RQ)

## Introduction
This is the official repository for the **Robotics Project 2025**. The project focuses on creating code for the movement of a robotic arm and allows us to identify and manipulate block of different types, placing them in designated locations regardless of their initial pose.

## Authors
- [@lucahardonk](https://github.com/lucahardonk)
- [@NickLech](https://github.com/NickLech)
- [@KeithLeoni](https://github.com/KeithLeoni)

---

## Description
This project has been developed for the course **Introduction to Robotics** at the **University of Trento, year 2024/2025**.

### Key Features:
- **Block Recognition**: Identify and classify 10 different types of blocks, even if flipped or placed on their sides.
- **Task Execution**: Move and stack blocks to create hardcoded structures like castles or towers.
---

# Documentation
## Move node
The move node is responsible for moving the robot in the simulation, it's written in C++ and it's responsible for computing the actual movement of the robot, while also checking the presence of singularities.

## Planning node
The planner node is responsible for creating the enviroment, deciding where each block has to be positioned and plann the path of the robot.

## Vision node
The vision node is responsible for detecting the blocks in the simulation, it's written in Python and its main purpose is to detect the blocks on the table and classify them, in order to understand where to put each one, and how to pick it up.

## For more details see the report
[Report](https://github.com/KeithLeoni/ros2_ws/blob/main/report.pdf)

---

## Installation

Before starting the simulation, make sure you have the repository downloaded to your workspace. To do this, clone the repository with the following command:

```bash
git clone https://github.com/KeithLeoni/ros2_ws.git
```

Navigate inside the directory
```bash
cd ros2_ws
```

## Starting the Simulation

Follow the steps below to launch and control the robot simulation.

### 1. Launch the Robot Nodes and Services

Before anything else, you need to launch the required nodes and services that bring up the robot's system. This will initialize all necessary components and services.

Run the following command to start the robot system:

```bash
ros2 launch arm_bringup bringup_launch.py
```

### 2. Run the planning node

Once the system is up and running, you can launch the planning node. This will allow the robot to perform the necessary planning operations.

```bash
ros2 run planning_pkg planning_node
```


