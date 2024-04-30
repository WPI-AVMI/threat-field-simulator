#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-lab-03@192.168.1.13 "export DISPLAY=:0; python /home/avmi-lab-03/Documents/Scripts/project.py"
