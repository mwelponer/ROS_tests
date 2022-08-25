#!/bin/bash

sudo -e systemctl disable systemd-timesyncd
sudo -e systemctl enable ntp 
sudo -e systemctl start ntp
