#!/bin/bash

cur_path=`pwd`

result=$(lsmod | grep videodev)
if [ "$result" = "" ];then
    sudo modprobe videodev
fi
usr_name=`whoami`
result=$(lsmod | grep altera_drv)
if [ "$result" = "" ];then
#Enter the driver file directory to install the driver file.
#In this example, the driver file is placed in the SensorFusion_alpha directory,
#and the user can specify it according to the actual driver file directory.
    cd driver
    sudo insmod altera_drv.ko
    cd $cur_path
    sudo chown $usr_name:$usr_name /dev/fpga_cdev
    sudo chown $usr_name:$usr_name /dev/rs16_dev
    sudo chmod 660 /dev/fpga_cdev
    sudo chmod 660 /dev/rs16_dev
fi
