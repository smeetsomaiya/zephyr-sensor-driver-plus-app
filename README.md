# Thread Programming and Device driver in Zephyr RTOS Galileo Gen2

###########

Overview
********
This program measures distance using HCSR04 sensor on Zephyr - Galileo
The driver is designed to run 2 HCSR04 devices simultaneously.

The sequence can be run by using "hcsr" command along with appropriate subcommands.


1. hcsr select [0/1]
This select which device should be used. For example, hcsr select 0 will select HCSR04_0. 
The configuration for each device is defined in the driver's KConfig file.

2. hcsr start [number_of_samples]
This starts the measurement on the selected device and collect samples equal to the value given by number_of_samples. For example, hcsr start 100

3. hcsr dump [p1] [p2]
This dumps the sample numbers p1 through p2 on the shell along with the elapsed time (time beginning from the first measurement). Note that p2 should be greater than or equal to p1.

Building and Running
********************
To build for the galileo and write the zephyr.strip image to an sdcard, execute the script provided in the root of the project.
	hcsr_app/run

Steps to execute the script -
1. sudo chmod +x measure80/run
2. ./run

The script assumes that your sdcard is named as "ZEPHYR", and it is mounted at /dev/mmcblk0p1. You can change it accordingly.
