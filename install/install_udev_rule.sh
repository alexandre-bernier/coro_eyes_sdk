#!/bin/bash

# Must run in sudo
if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo:"
    echo "sudo $0 $*"
    exit 1
fi

# Check if udev/rules.d directory exists
if [ ! -d "/etc/udev/rules.d" ]; then
    echo "Can't find udev directory. Make sure you installed udev before running this script."
    exit 2
fi

# Create udev rule
file="/etc/udev/rules.d/40-lightcrafter.rules"
echo "KERNEL==\"hidraw*\", ATTRS{busnum}==\"1\", ATTRS{idVendor}==\"0451\", ATTRS{idProduct}==\"6401\", GROUP=\"plugdev\", MODE=\"0666\"" > $file
cat $file
