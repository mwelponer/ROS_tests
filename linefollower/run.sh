#!/bin/bash

source /home/$USER/catkin_ws/devel/setup.bash
cd /home/$USER/catkin_ws
catkin_make
rosrun line_follower line_follower
