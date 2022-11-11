#!/bin/bash

if [ "$#" -ne 1 ]; then 
    echo "Usage: network_set.sh master"
    exit 2
fi


host=`ip a |grep 192.168. | awk '{print $2}'`
host_ip=${host::-3}
master=$1

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

