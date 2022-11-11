#!/bin/bash

cam=`rosnode list | grep realsense | head -n 1`
#echo $cam/reset
rosservice call $cam/reset
