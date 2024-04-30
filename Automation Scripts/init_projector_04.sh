#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-lab-04@192.168.1.14 "export DISPLAY=:0; python /home/avmi-lab-04/Documents/Scripts/project.py"
