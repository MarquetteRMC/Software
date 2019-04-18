#!/bin/bash

source ~/Software/ros/devel/setup.bash

screen -dmS roscore

sleep 2

screen -dmS rosrun odrive_ros odrive_node

screen -dmS rosrun qr_detect qr_detect_1

screen -dmS rosrun current_sensing 30A_current_sensor.py

screen -dmS rosrun phidgets_imu phidgets_imu_node

screen -dmS source ~/Software/scripts/openni.sh

#screen -dmS rosrun roboclaw_node roboclaw_node.py

#screen -dmS rosrun current_sensing voltageinput.py

#screen -dmS rosrun command_vel my_command_vels
