#!/bin/sh -l
# launcher.sh

echo exec loader
stty -F /dev/ttyUSB0 115200 #setup px4 flow 
stty -F /dev/ttyO2 115200  #setup baro serial
python /home/QV2/loader.py
