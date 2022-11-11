#!/bin/bash

if [[ $(rosnode list |grep raspicam) ]]; then
    echo "raspicam and web_video_server active, killing nodes"
    source ./leo_kill_cam_web.sh
    echo "done."
else
    echo "no raspicam and web_video_server nodes"
fi

if [[ $(rosnode list |grep realsense) ]]; then
    echo "realsense is active, killing node"
    source ./leo_kill_realsense.sh
    echo "done."
else
    echo "no realsense node"
fi

echo "launch realsense.launch"
roslaunch rtabmap_bringup realsense.launch


# disable video web server & rgb camera
#source ./leo_kill_cam_web.sh

# disable realsense
#source ./leo_kill_realsense.sh

#roslaunch rtabmap_bringup realsense.launch

