#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-cam-sys@192.168.1.16 "sudo su <<EOF; cd ros2_ws; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run avmi_lab_cam_sys cam_sys; EOF"
