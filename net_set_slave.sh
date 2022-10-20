#!/bin/bash

source ~/catkin_ws/src/net_reset.sh
echo "export ROS_MASTER_URI=http://10.0.0.23:11311/ # Master on PC" >> ~/.bashrc
echo "export ROS_HOSTNAME=leo" >> ~/.bashrc
echo "export ROS_IP=10.0.0.1" >> ~/.bashrc

