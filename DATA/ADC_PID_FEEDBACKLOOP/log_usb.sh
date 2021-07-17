#!/bin/bash
###
# Leigh Oliver 2021
# A small script that logs Serial console to file, using 'screen' command.
# To exit, press [Ctrl+A]-K (Ctrl+A enters command mode, 'K' triggers kill of screen)
###

### Define defaults
port=/dev/ttyUSB0
baudRate=115200
unset fileName

### Get command arguments, mainly output file name. There is also defaults for interface and baud rate
while getopts p:b:f: flag
do
    case "${flag}" in
        p) port=${OPTARG};;
        b) baudRate=${OPTARG};;
        f) fileName=${OPTARG};;
    esac
done

### Check for filename
if [ -z "$fileName" ]
then
   echo "usage: ./log_usb.sh -f outfile.log"
   exit
fi

echo $fileName
echo $port
echo $baudRate

screen -L -Logfile $fileName $port $baudRate
