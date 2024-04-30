#!/bin/bash

sshpass -p 'letmein' ssh -f avmi-lab-02@192.168.1.12 "export DISPLAY=:0; python /home/avmi-lab-02/Documents/Scripts/project.py"
