#!/bin/bash

source ~/networkReset.sh
echo "export ROS_MASTER_URI=http://10.0.0.1:11311/ # Master on Leo" >> ~/.bashrc
echo "export ROS_HOSTNAME=10.0.0.1" >> ~/.bashrc
echo "export ROS_IP=10.0.0.1" >> ~/.bashrc

