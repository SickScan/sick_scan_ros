# sick_scan_ros based on ssbl

This stack provides a ROS driver for the SICK lidar and radar sensors mentioned in the following list.

## Table of Contents

- [Supported Hardware](#supported-hardware)
- [Start node](#start-node)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Tools](#tools)
- [Troubleshooting](#troubleshooting)
- [SLAM-Support](doc/slam.md)
- [Radar](doc/radar.md)
- [Profiling](doc/profiling.md)
- [Testing](#testing)
- [Creators](#creators)

## Supported Hardware

This driver should work with all of the following products.

ROS Device Driver for SICK lidar and radar sensors - supported scanner types:



| **device name**    |  **part no.**                                                                                                                | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| TiM551             | [1060445](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim551-2050001/p/p343045)                 | 1 layer max. range: 10 m, ang. resol. 1.00[deg] | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM561             | [1071419](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim561-2050101/p/p369446)                 | 1 layer max. range: 10 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM571             | [1079742](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781             | [1096807](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781-2174101/p/p594148)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |

##  Start Node

Use the following command to start ROS node:

- For TiM5xx-family:
```bash
roslaunch sick_scan_ros sick_tim_5xx.launch
```

### Starting Scanner with Specific Ip Address
To start the scanner with a specific IP address, the launch command can be used for most launch files as follows.
The hostname is the ip-address of the scanner:

```bash
roslaunch sick_scan_ros sick_tim_5xx.launch hostname:=<ip-address>
```
e.g.
```bash
roslaunch  sick_scan_ros sick_tim_5xx.launch hostname:=192.168.0.71
```

## Troubleshooting

1. Check Scanner IP in the launch file.
2. Check Ethernet connection to scanner with netcat e.g. ```nc -z -v -w5 $SCANNERIPADDRESS 2112```.
3. View node startup output wether the IP connection could be established
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt
6. If you stop the scanner in your debugging IDE or by other hard interruption (like Ctrl-C), you must wait until 60 sec. before
   the scanner is up and running again. During this time the MRS6124 reconnects twice.
   If you do not wait this waiting time you could see one of the following messages:
   * TCP connection error
   * Error-Message 0x0d
7. Amplitude values in rviz: If you see only one color in rviz try the following:
   Set the min/max-Range of intensity display in the range [0...200] and switch on the intensity flag in the launch file  
8. In case of network problems check your own ip address and the ip address of your laser scanner (by using SOPAS ET).
   * List of own IP-addresses: ifconfig|grep "inet addr"
   * Try to ping scanner ip address (used in launch file)
9. If the driver stops during init phase please stop the driver with ctrl-c and restart (could be caused due to protocol ASCII/Binary cola-dialect).

## Support

* In case of technical support please open a new issue. For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,						
  are closed by us because we assume that the user has solved the problem.


## Installation

### From Source

```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone -b devel --single-branch git://github.com/SickScan/sick_scan_ros.git
cd ..
catkin_make install
source ~/ros_catkin_ws/install/setup.bash
```

## Quick Start

```bash
roslaunch sick_scan_ros sick_tim_5xx.launch
rosrun rviz rviz
```

## Keywords
TiM5xx
TiM551
TiM561
TiM571


## Creators

**Michael Lehning**

- <http://www.lehning.de>

on behalf of SICK AG

- <http://www.sick.com>

------------------------------------------------------------------------

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png" width="420">

![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")
