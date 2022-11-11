#!/bin/bash

#rosservice call /$1/realsense2_camera/reset

# disable video web server & rgb camera
source ./leo_kill_cam_web.sh
# disable realsense
source ./leo_kill_realsense.sh

roslaunch rtabmap_bringup realsense.launch

