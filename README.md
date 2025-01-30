# RoboticsProject

## Introduction
This is the official repository for the **Robotics Project 2025**. The project focuses on creating code for the movement of a robotic arm and allows us to identify and manipulate block of different types, placing them in designated locations regardless of their initial orientation.

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

---

## Installation
---
## Prerequisites
- **Docker**: Ensure Docker is installed and running on your system.
- **Docker Network Setup**: Create a Docker network named `ursim_net` before running the containers:
  ```bash
  docker network create --subnet=192.168.56.0/24 ursim_net
  ```



