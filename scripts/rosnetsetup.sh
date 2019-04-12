#!/bin/bash

MY_IP=$(ifconfig | grep 'inet '| grep -v '127.0.0.1' | tail -1 | awk '{print $2}')

export ROS_IP=$MY_IP
export ROS_HOSTNAME=$MY_IP
export ROS_MASTER_URI=http://192.168.1.132:11311 #on robot should be localhost

echo "ROS_HOSTNAME: "$ROS_HOSTNAME
echo "ROS_IP: "$ROS_IP
echo "ROS_MASTER_URI: "$ROS_MASTER_URI



