#!/bin/bash

# Source the ROS2 workspace
source ~/ros2_ws/install/setup.bash

# Activate the virtual environment
source /home/ubuntu/ros2_env/bin/activate



# Set up the environment variables (if needed)
#export PYTHONPATH=/home/ubuntu/ros2_env/lib/python3.12/site-packages
export PATH=/home/ubuntu/ros2_env/bin:$PATH

# Define the base path for the Python scripts
POSE_ESTIMATOR_PATH="$HOME/ros2_ws/src/pose_estimator_pkg/pose_estimator_pkg"

cd "$POSE_ESTIMATOR_PATH"

# Run each Python script in the background
./pt_inference_node.py &
./visualization_node.py &
#./icp_pose_estimator.py &

# Wait for all background processes to finish
wait

