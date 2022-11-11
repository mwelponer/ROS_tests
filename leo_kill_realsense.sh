#!/bin/bash

rosnode list | grep realsense | while read line ; do rosnode kill $line ; done
