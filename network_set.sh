#!/bin/bash

if [ "$#" -ne 2 ]; then 
    echo "Usage: network_set.sh host_ip master"
    exit 2
fi

host_ip=$1
master=$2

hostname=$(hostname)
master_ip=`nslookup $master | grep 192.168 | cut -d' ' -f2-`
source ./net_reset.sh
echo "export ROS_MASTER_URI=http://$master:11311/ # Master on Leo" >> ~/.bashrc
echo "export ROS_HOSTNAME=$hostname" >> ~/.bashrc
echo "export ROS_IP=$host_ip" >> ~/.bashrc
source ~/.bashrc

#tail ~/.bashrc

echo "hostname : $hostname ($host_ip)"
echo "master   : $master ($master_ip)"

