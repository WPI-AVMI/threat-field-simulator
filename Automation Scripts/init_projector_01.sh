#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-lab-01@192.168.1.11 "export DISPLAY=:0; python /home/avmi-lab-01/Documents/Scripts/project.py"
