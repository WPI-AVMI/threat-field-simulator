#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-cam-sys@192.168.1.16 "pkill cam_sys"
