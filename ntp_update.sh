#!/bin/bash

#sudo systemctl daemon-reload
#sudo timedatectl set-ntp off
#sudo timedatectl set-ntp on

sudo service ntp stop
sudo ntpdate -s time.nist.gov
sudo service ntp start

timedatectl status
